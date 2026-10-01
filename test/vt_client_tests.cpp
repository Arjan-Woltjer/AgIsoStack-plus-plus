#include <gtest/gtest.h>

#include "isobus//utility/iop_file_interface.hpp"
#include "isobus/hardware_integration/can_hardware_interface.hpp"
#include "isobus/hardware_integration/virtual_can_plugin.hpp"
#include "isobus/isobus/can_general_parameter_group_numbers.hpp"
#include "isobus/isobus/can_network_manager.hpp"
#include "isobus/isobus/isobus_virtual_terminal_client.hpp"
#include "isobus/utility/system_timing.hpp"

#include "helpers/control_function_helpers.hpp"
#include "helpers/test_fixture.hpp"

using namespace isobus;

class DerivedTestVTClient : public VirtualTerminalClient
{
public:
	DerivedTestVTClient(std::shared_ptr<PartneredControlFunction> partner, std::shared_ptr<InternalControlFunction> clientSource) :
	  VirtualTerminalClient(partner, clientSource) {
		  // Does nothing
	  };

	void test_wrapper_process_rx_message(const CANMessage &message, void *parentPointer)
	{
		VirtualTerminalClient::process_rx_message(message, parentPointer);
	}

	bool test_wrapper_get_any_pool_needs_scaling() const
	{
		return VirtualTerminalClient::get_any_pool_needs_scaling();
	}

	bool test_wrapper_scale_object_pools()
	{
		return VirtualTerminalClient::scale_object_pools();
	}

	bool test_wrapper_get_is_object_scalable(VirtualTerminalObjectType type) const
	{
		return VirtualTerminalClient::get_is_object_scalable(type);
	}

	FontSize test_wrapper_get_font_or_next_smallest_font(FontSize originalFont) const
	{
		return VirtualTerminalClient::get_font_or_next_smallest_font(originalFont);
	}

	FontSize test_wrapper_remap_font_to_scale(FontSize originalFont, float scaleFactor) const
	{
		return VirtualTerminalClient::remap_font_to_scale(originalFont, scaleFactor);
	}

	std::uint32_t test_wrapper_get_minimum_object_length(VirtualTerminalObjectType type) const
	{
		return VirtualTerminalClient::get_minimum_object_length(type);
	}

	std::uint32_t test_wrapper_get_number_bytes_in_object(std::uint8_t *buffer)
	{
		return VirtualTerminalClient::get_number_bytes_in_object(buffer);
	}

	bool test_wrapper_resize_object(std::uint8_t *buffer, float scaleFactor, VirtualTerminalObjectType type)
	{
		return resize_object(buffer, scaleFactor, type);
	}

	void test_wrapper_set_supported_fonts(std::uint8_t smallFontsBitfield, std::uint8_t largeFontsBitfield)
	{
		smallFontSizesBitfield = smallFontsBitfield;
		largeFontSizesBitfield = largeFontsBitfield;
	}

	void test_wrapper_set_state(VirtualTerminalClient::StateMachineState value)
	{
		VirtualTerminalClient::set_state(value);
	}

	void test_wrapper_set_screen(std::uint16_t dataMaskPixels, std::uint8_t softKeyPixels)
	{
		xPixels = dataMaskPixels;
		yPixels = dataMaskPixels;
		softKeyXAxisPixels = softKeyPixels;
		softKeyYAxisPixels = softKeyPixels;
	}

	const std::vector<std::uint8_t> &test_wrapper_get_scaled_pool(std::uint8_t poolIndex) const
	{
		return objectPools.at(poolIndex).scaledObjectPool;
	}

	static std::vector<std::uint8_t> staticTestPool;

	static bool testWrapperDataChunkCallback(std::uint32_t,
	                                         std::uint32_t bytesOffset,
	                                         std::uint32_t numberOfBytesNeeded,
	                                         std::uint8_t *chunkBuffer,
	                                         void *)
	{
		memcpy(chunkBuffer, &staticTestPool.data()[bytesOffset], numberOfBytesNeeded);
		return true;
	}

	void test_wrapper_process_command_queue()
	{
		VirtualTerminalClient::process_command_queue();
	}
};

std::vector<std::uint8_t> DerivedTestVTClient::staticTestPool;

class VirtualTerminalTest : public AgIsoStackTestFixture
{
	// Wrapper to give tests a more meaningful name - no content.
};

TEST_F(VirtualTerminalTest, InitializeAndInitialState)
{
	NAME clientNAME(0);
	auto internalECU = CANNetworkManager::CANNetwork.create_internal_control_function(clientNAME, 0, 0x26);

	std::vector<isobus::NAMEFilter> vtNameFilters;
	const isobus::NAMEFilter testFilter(isobus::NAME::NAMEParameters::FunctionCode, static_cast<std::uint8_t>(isobus::NAME::Function::VirtualTerminal));
	vtNameFilters.push_back(testFilter);

	auto vtPartner = CANNetworkManager::CANNetwork.create_partnered_control_function(0, vtNameFilters);

	DerivedTestVTClient clientUnderTest(vtPartner, internalECU);

	EXPECT_EQ(false, clientUnderTest.get_is_initialized());
	EXPECT_EQ(false, clientUnderTest.get_is_connected());

	clientUnderTest.initialize(false);

	EXPECT_EQ(true, clientUnderTest.get_is_initialized());

	clientUnderTest.initialize(false);

	EXPECT_EQ(true, clientUnderTest.get_is_initialized()); // Double init should be at least tolerated

	EXPECT_EQ(false, clientUnderTest.get_has_adjustable_volume_output());
	EXPECT_EQ(false, clientUnderTest.get_multiple_frequency_audio_output());
	EXPECT_EQ(false, clientUnderTest.get_support_pointing_device_with_pointing_message());
	EXPECT_EQ(false, clientUnderTest.get_support_touchscreen_with_pointing_message());
	EXPECT_EQ(false, clientUnderTest.get_support_intermediate_coordinates_during_drag_operations());
	EXPECT_EQ(0, clientUnderTest.get_number_y_pixels());
	EXPECT_EQ(0, clientUnderTest.get_number_x_pixels());
	EXPECT_EQ(VirtualTerminalClient::VTVersion::ReservedOrUnknown, clientUnderTest.get_connected_vt_version());

	EXPECT_NE(nullptr, clientUnderTest.get_internal_control_function());
	EXPECT_NE(nullptr, clientUnderTest.get_partner_control_function());

	clientUnderTest.terminate();
	CANNetworkManager::CANNetwork.deactivate_control_function(vtPartner);
	CANNetworkManager::CANNetwork.deactivate_control_function(internalECU);
}

TEST_F(VirtualTerminalTest, VTStatusMessage)
{
	NAME clientNAME(0);
	auto internalECU = CANNetworkManager::CANNetwork.create_internal_control_function(clientNAME, 0, 0x26);

	std::vector<isobus::NAMEFilter> vtNameFilters;
	const isobus::NAMEFilter testFilter(isobus::NAME::NAMEParameters::FunctionCode, static_cast<std::uint8_t>(isobus::NAME::Function::VirtualTerminal));
	vtNameFilters.push_back(testFilter);

	auto vtPartner = CANNetworkManager::CANNetwork.create_partnered_control_function(0, vtNameFilters);

	DerivedTestVTClient clientUnderTest(vtPartner, internalECU);

	EXPECT_EQ(NULL_OBJECT_ID, clientUnderTest.get_visible_data_mask());
	EXPECT_EQ(NULL_OBJECT_ID, clientUnderTest.get_visible_soft_key_mask());

	CANIdentifier identifier(CANIdentifier::Type::Extended, static_cast<std::uint32_t>(CANLibParameterGroupNumber::VirtualTerminalToECU), CANIdentifier::CANPriority::PriorityDefault6, 0, 0);
	CANMessage testMessage(CANMessage::Type::Receive,
	                       identifier,
	                       {
	                         0xFE, // VT Status message function code
	                         0x26, // Working set master address
	                         1234 & 0xFF, // Data mask active
	                         1234 >> 8, // Data mask active
	                         4567 & 0xFF, // Soft key mask active
	                         4567 >> 8, // Soft key mask active
	                         0xFF, // Busy codes
	                         1, // VT Function code that is being executed
	                       },
	                       nullptr,
	                       nullptr,
	                       0);

	clientUnderTest.test_wrapper_process_rx_message(testMessage, &clientUnderTest);

	EXPECT_EQ(1234, clientUnderTest.get_visible_data_mask());
	EXPECT_EQ(4567, clientUnderTest.get_visible_soft_key_mask());
	EXPECT_EQ(0xFE, clientUnderTest.get_active_working_set_master_address()); // Expect null address since not in the connected state

	// Test the master address is correct when in the connected state
	clientUnderTest.test_wrapper_set_state(VirtualTerminalClient::StateMachineState::Connected);
	EXPECT_EQ(0x26, clientUnderTest.get_active_working_set_master_address());

	CANNetworkManager::CANNetwork.deactivate_control_function(vtPartner);
	CANNetworkManager::CANNetwork.deactivate_control_function(internalECU);
}

TEST_F(VirtualTerminalTest, FullPoolAutoscalingWithVector)
{
	NAME clientNAME(0);
	clientNAME.set_arbitrary_address_capable(true);
	clientNAME.set_industry_group(1);
	clientNAME.set_device_class(0);
	clientNAME.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::OilSystemMonitor));
	clientNAME.set_identity_number(1);
	clientNAME.set_ecu_instance(1);
	clientNAME.set_function_instance(0);
	clientNAME.set_device_class_instance(0);
	clientNAME.set_manufacturer_code(69);

	auto internalECU = CANNetworkManager::CANNetwork.create_internal_control_function(clientNAME, 0, 0x26);

	std::vector<isobus::NAMEFilter> vtNameFilters;
	const isobus::NAMEFilter testFilter(isobus::NAME::NAMEParameters::FunctionCode, static_cast<std::uint8_t>(isobus::NAME::Function::VirtualTerminal));
	vtNameFilters.push_back(testFilter);

	auto vtPartner = CANNetworkManager::CANNetwork.create_partnered_control_function(0, vtNameFilters);

	DerivedTestVTClient clientUnderTest(vtPartner, internalECU);

	// Actual tests start here
	std::vector<std::uint8_t> testPool = isobus::IOPFileInterface::read_iop_file("../../examples/virtual_terminal/version3_object_pool/VT3TestPool.iop");

	if (0 == testPool.size())
	{
		// Try a different path to mitigate differences between how IDEs run the unit test
		testPool = isobus::IOPFileInterface::read_iop_file("../examples/virtual_terminal/version3_object_pool/VT3TestPool.iop");
	}

	EXPECT_NE(0, testPool.size());

	clientUnderTest.set_object_pool(0, &testPool);

	EXPECT_EQ(false, clientUnderTest.test_wrapper_get_any_pool_needs_scaling());

	// Test invalid soft key width
	clientUnderTest.set_object_pool_scaling(0, 64, 0);

	EXPECT_EQ(false, clientUnderTest.test_wrapper_get_any_pool_needs_scaling());

	clientUnderTest.set_object_pool_scaling(0, 240, 240);

	// Check functionality of get_any_pool_needs_scaling
	EXPECT_EQ(true, clientUnderTest.test_wrapper_get_any_pool_needs_scaling());

	// Full scaling test using the example pool
	EXPECT_EQ(true, clientUnderTest.test_wrapper_scale_object_pools());

	CANNetworkManager::CANNetwork.deactivate_control_function(vtPartner);
	CANNetworkManager::CANNetwork.deactivate_control_function(internalECU);
}

TEST_F(VirtualTerminalTest, FullPoolAutoscalingWithDataChunkCallbacks)
{
	NAME clientNAME(0);
	clientNAME.set_arbitrary_address_capable(true);
	clientNAME.set_industry_group(1);
	clientNAME.set_device_class(0);
	clientNAME.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::OilSystemMonitor));
	clientNAME.set_identity_number(1);
	clientNAME.set_ecu_instance(1);
	clientNAME.set_function_instance(0);
	clientNAME.set_device_class_instance(0);
	clientNAME.set_manufacturer_code(69);

	auto internalECU = CANNetworkManager::CANNetwork.create_internal_control_function(clientNAME, 0, 0x26);

	std::vector<isobus::NAMEFilter> vtNameFilters;
	const isobus::NAMEFilter testFilter(isobus::NAME::NAMEParameters::FunctionCode, static_cast<std::uint8_t>(isobus::NAME::Function::VirtualTerminal));
	vtNameFilters.push_back(testFilter);

	auto vtPartner = CANNetworkManager::CANNetwork.create_partnered_control_function(0, vtNameFilters);

	DerivedTestVTClient clientUnderTest(vtPartner, internalECU);

	// Actual tests start here
	DerivedTestVTClient::staticTestPool = isobus::IOPFileInterface::read_iop_file("../../examples/virtual_terminal/version3_object_pool/VT3TestPool.iop");

	if (0 == DerivedTestVTClient::staticTestPool.size())
	{
		// Try a different path to mitigate differences between how IDEs run the unit test
		DerivedTestVTClient::staticTestPool = isobus::IOPFileInterface::read_iop_file("../examples/virtual_terminal/version3_object_pool/VT3TestPool.iop");
	}

	EXPECT_NE(0, DerivedTestVTClient::staticTestPool.size());

	clientUnderTest.register_object_pool_data_chunk_callback(0, DerivedTestVTClient::staticTestPool.size(), DerivedTestVTClient::testWrapperDataChunkCallback);

	clientUnderTest.set_object_pool_scaling(0, 240, 240);

	// Check functionality of get_any_pool_needs_scaling
	EXPECT_EQ(true, clientUnderTest.test_wrapper_get_any_pool_needs_scaling());

	// Full scaling test using the example pool
	EXPECT_EQ(true, clientUnderTest.test_wrapper_scale_object_pools());

	CANNetworkManager::CANNetwork.deactivate_control_function(vtPartner);
	CANNetworkManager::CANNetwork.deactivate_control_function(internalECU);
}

namespace
{
	void append_u16(std::vector<std::uint8_t> &pool, std::uint16_t value)
	{
		pool.push_back(static_cast<std::uint8_t>(value & 0xFF));
		pool.push_back(static_cast<std::uint8_t>(value >> 8));
	}

	void append_font_attributes(std::vector<std::uint8_t> &pool, std::uint16_t id, VirtualTerminalClient::FontSize size)
	{
		append_u16(pool, id);
		pool.push_back(static_cast<std::uint8_t>(VirtualTerminalObjectType::FontAttributes));
		pool.push_back(1); // Colour
		pool.push_back(static_cast<std::uint8_t>(size));
		pool.push_back(0); // Font type
		pool.push_back(0); // Font style
		pool.push_back(0); // Number of macros
	}

	void append_output_string(std::vector<std::uint8_t> &pool, std::uint16_t id, std::uint16_t width, std::uint16_t fontAttributes)
	{
		append_u16(pool, id);
		pool.push_back(static_cast<std::uint8_t>(VirtualTerminalObjectType::OutputString));
		append_u16(pool, width);
		append_u16(pool, 40); // Height
		pool.push_back(0); // Background colour
		append_u16(pool, fontAttributes);
		pool.push_back(0); // Options
		append_u16(pool, 0xFFFF); // No variable reference
		pool.push_back(1); // Justification
		append_u16(pool, 4); // Length
		for (char c : { 'T', 'E', 'X', 'T' })
		{
			pool.push_back(static_cast<std::uint8_t>(c));
		}
		pool.push_back(0); // Number of macros
	}

	void append_key(std::vector<std::uint8_t> &pool, std::uint16_t id, std::uint16_t child)
	{
		append_u16(pool, id);
		pool.push_back(static_cast<std::uint8_t>(VirtualTerminalObjectType::Key));
		pool.push_back(0); // Background colour
		pool.push_back(1); // Key code
		pool.push_back(1); // Number of objects
		pool.push_back(0); // Number of macros
		append_u16(pool, child);
		append_u16(pool, 0); // X
		append_u16(pool, 0); // Y
	}

	// The object with this ID in an uploaded pool: its offset, or -1.
	int find_object(const std::vector<std::uint8_t> &pool, std::uint16_t id, DerivedTestVTClient &client)
	{
		std::vector<std::uint8_t> copy(pool);
		std::size_t offset = 0;
		while (offset + 3 <= copy.size())
		{
			if ((copy[offset] | (copy[offset + 1] << 8)) == id)
			{
				return static_cast<int>(offset);
			}
			std::uint32_t size = client.test_wrapper_get_number_bytes_in_object(&copy[offset]);
			if (0 == size)
			{
				break;
			}
			offset += size;
		}
		return -1;
	}
} // namespace

// A Key is sized by the soft key designator, but the label it shows is a separate object in
// the pool, and so is that label's font. Scaled with the data mask factor, a label on a
// 60 px key designed for a 200 px mask grows to 144 px and a much larger font on a 480 px
// mask with 64 px keys, and spills off the key. Objects a Key shows, and fonts only they use,
// must follow the soft key factor.
TEST_F(VirtualTerminalTest, AutoscalingSoftKeyLabelsFollowTheKey)
{
	NAME clientNAME(0);
	clientNAME.set_arbitrary_address_capable(true);
	clientNAME.set_industry_group(1);
	clientNAME.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::OilSystemMonitor));
	clientNAME.set_identity_number(2);
	clientNAME.set_manufacturer_code(69);
	auto internalECU = CANNetworkManager::CANNetwork.create_internal_control_function(clientNAME, 0, 0x27);
	std::vector<isobus::NAMEFilter> vtNameFilters = { isobus::NAMEFilter(isobus::NAME::NAMEParameters::FunctionCode, static_cast<std::uint8_t>(isobus::NAME::Function::VirtualTerminal)) };
	auto vtPartner = CANNetworkManager::CANNetwork.create_partnered_control_function(0, vtNameFilters);
	DerivedTestVTClient clientUnderTest(vtPartner, internalECU);

	constexpr std::uint16_t KEY_FONT = 100;
	constexpr std::uint16_t MASK_FONT = 101;
	constexpr std::uint16_t KEY_LABEL = 200;
	constexpr std::uint16_t MASK_LABEL = 201;
	constexpr std::uint16_t KEY_LABEL_SHARED_FONT = 202;
	std::vector<std::uint8_t> pool;
	append_font_attributes(pool, KEY_FONT, VirtualTerminalClient::FontSize::Size8x8);
	append_font_attributes(pool, MASK_FONT, VirtualTerminalClient::FontSize::Size8x8);
	append_output_string(pool, KEY_LABEL, 60, KEY_FONT);
	append_output_string(pool, MASK_LABEL, 60, MASK_FONT);
	append_output_string(pool, KEY_LABEL_SHARED_FONT, 60, MASK_FONT);
	append_key(pool, 300, KEY_LABEL);
	append_key(pool, 301, KEY_LABEL_SHARED_FONT);

	clientUnderTest.set_object_pool(0, &pool);
	clientUnderTest.set_object_pool_scaling(0, 200, 60);
	clientUnderTest.test_wrapper_set_supported_fonts(0xFF, 0xFF);
	clientUnderTest.test_wrapper_set_screen(480, 64);
	ASSERT_TRUE(clientUnderTest.test_wrapper_scale_object_pools());

	const std::vector<std::uint8_t> &scaled = clientUnderTest.test_wrapper_get_scaled_pool(0);
	auto width_of = [&](std::uint16_t id) {
		int offset = find_object(scaled, id, clientUnderTest);
		EXPECT_GE(offset, 0);
		return (offset < 0) ? 0 : (scaled[offset + 3] | (scaled[offset + 4] << 8));
	};
	auto font_size_of = [&](std::uint16_t id) {
		int offset = find_object(scaled, id, clientUnderTest);
		EXPECT_GE(offset, 0);
		return (offset < 0) ? VirtualTerminalClient::FontSize::Size6x8 : static_cast<VirtualTerminalClient::FontSize>(scaled[offset + 4]);
	};

	// Labels on keys: the soft key factor (64 / 60), so they still fit the 64 px key.
	EXPECT_EQ(64, width_of(KEY_LABEL));
	EXPECT_EQ(64, width_of(KEY_LABEL_SHARED_FONT));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size8x8, font_size_of(KEY_FONT));
	// The data mask label and its font: the data mask factor (480 / 200), as before.
	EXPECT_EQ(144, width_of(MASK_LABEL));
	EXPECT_NE(VirtualTerminalClient::FontSize::Size8x8, font_size_of(MASK_FONT));

	// Scaling up a little keeps the small fonts; their tables used to fall through to 6x8.
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size8x8, clientUnderTest.test_wrapper_remap_font_to_scale(VirtualTerminalClient::FontSize::Size8x8, 1.2f));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size8x12, clientUnderTest.test_wrapper_remap_font_to_scale(VirtualTerminalClient::FontSize::Size8x12, 1.2f));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size6x8, clientUnderTest.test_wrapper_remap_font_to_scale(VirtualTerminalClient::FontSize::Size8x8, 0.5f));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size6x8, clientUnderTest.test_wrapper_remap_font_to_scale(VirtualTerminalClient::FontSize::Size8x12, 0.5f));

	CANNetworkManager::CANNetwork.deactivate_control_function(vtPartner);
	CANNetworkManager::CANNetwork.deactivate_control_function(internalECU);
}

TEST_F(VirtualTerminalTest, FullPoolAutoscalingWithPointer)
{
	NAME clientNAME(0);
	clientNAME.set_arbitrary_address_capable(true);
	clientNAME.set_industry_group(1);
	clientNAME.set_device_class(0);
	clientNAME.set_function_code(static_cast<std::uint8_t>(isobus::NAME::Function::OilSystemMonitor));
	clientNAME.set_identity_number(1);
	clientNAME.set_ecu_instance(1);
	clientNAME.set_function_instance(0);
	clientNAME.set_device_class_instance(0);
	clientNAME.set_manufacturer_code(69);

	auto internalECU = CANNetworkManager::CANNetwork.create_internal_control_function(clientNAME, 0, 0x26);

	std::vector<isobus::NAMEFilter> vtNameFilters;
	const isobus::NAMEFilter testFilter(isobus::NAME::NAMEParameters::FunctionCode, static_cast<std::uint8_t>(isobus::NAME::Function::VirtualTerminal));
	vtNameFilters.push_back(testFilter);

	auto vtPartner = CANNetworkManager::CANNetwork.create_partnered_control_function(0, vtNameFilters);

	DerivedTestVTClient clientUnderTest(vtPartner, internalECU);

	// Actual tests start here
	std::vector<std::uint8_t> testPool = isobus::IOPFileInterface::read_iop_file("../../examples/virtual_terminal/version3_object_pool/VT3TestPool.iop");

	if (0 == testPool.size())
	{
		// Try a different path to mitigate differences between how IDEs run the unit test
		testPool = isobus::IOPFileInterface::read_iop_file("../examples/virtual_terminal/version3_object_pool/VT3TestPool.iop");
	}

	EXPECT_NE(0, testPool.size());

	clientUnderTest.set_object_pool(0, testPool.data(), testPool.size());

	EXPECT_EQ(false, clientUnderTest.test_wrapper_get_any_pool_needs_scaling());

	// Test invalid soft key width
	clientUnderTest.set_object_pool_scaling(0, 64, 0);

	EXPECT_EQ(false, clientUnderTest.test_wrapper_get_any_pool_needs_scaling());

	// Test invalid data mask key width
	clientUnderTest.set_object_pool_scaling(0, 0, 64);

	EXPECT_EQ(false, clientUnderTest.test_wrapper_get_any_pool_needs_scaling());

	clientUnderTest.set_object_pool_scaling(0, 240, 240);

	// Check functionality of get_any_pool_needs_scaling
	EXPECT_EQ(true, clientUnderTest.test_wrapper_get_any_pool_needs_scaling());

	// Full scaling test using the example pool
	EXPECT_EQ(true, clientUnderTest.test_wrapper_scale_object_pools());

	CANNetworkManager::CANNetwork.deactivate_control_function(vtPartner);
	CANNetworkManager::CANNetwork.deactivate_control_function(internalECU);
}

TEST_F(VirtualTerminalTest, ObjectMetadataTests)
{
	NAME clientNAME(0);
	auto internalECU = CANNetworkManager::CANNetwork.create_internal_control_function(clientNAME, 0, 0x26);

	std::vector<isobus::NAMEFilter> vtNameFilters;
	const isobus::NAMEFilter testFilter(isobus::NAME::NAMEParameters::FunctionCode, static_cast<std::uint8_t>(isobus::NAME::Function::VirtualTerminal));
	vtNameFilters.push_back(testFilter);

	auto vtPartner = CANNetworkManager::CANNetwork.create_partnered_control_function(0, vtNameFilters);

	DerivedTestVTClient clientUnderTest(vtPartner, internalECU);

	// These values come from the ISO standard directly
	EXPECT_EQ(10, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::WorkingSet));
	EXPECT_EQ(8, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::DataMask));
	EXPECT_EQ(10, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::AlarmMask));
	EXPECT_EQ(10, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::Container));
	EXPECT_EQ(6, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::SoftKeyMask));
	EXPECT_EQ(7, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::Key));
	EXPECT_EQ(13, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::Button));
	EXPECT_EQ(13, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::InputBoolean));
	EXPECT_EQ(19, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::InputString));
	EXPECT_EQ(38, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::InputNumber));
	EXPECT_EQ(13, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::InputList));
	EXPECT_EQ(17, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::OutputString));
	EXPECT_EQ(29, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::OutputNumber));
	EXPECT_EQ(12, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::OutputList));
	EXPECT_EQ(11, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::OutputLine));
	EXPECT_EQ(13, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::OutputRectangle));
	EXPECT_EQ(15, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::OutputEllipse));
	EXPECT_EQ(14, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::OutputPolygon));
	EXPECT_EQ(21, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::OutputMeter));
	EXPECT_EQ(24, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::OutputLinearBarGraph));
	EXPECT_EQ(27, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::OutputArchedBarGraph));
	EXPECT_EQ(17, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::PictureGraphic));
	EXPECT_EQ(7, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::NumberVariable));
	EXPECT_EQ(5, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::StringVariable));
	EXPECT_EQ(8, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::FontAttributes));
	EXPECT_EQ(8, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::LineAttributes));
	EXPECT_EQ(8, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::FillAttributes));
	EXPECT_EQ(7, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::InputAttributes));
	EXPECT_EQ(5, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::ExtendedInputAttributes));
	EXPECT_EQ(5, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::ObjectPointer));
	EXPECT_EQ(5, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::Macro));
	EXPECT_EQ(6, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::ColourMap));
	EXPECT_EQ(34, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::GraphicsContext));
	EXPECT_EQ(17, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::WindowMask));
	EXPECT_EQ(10, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::KeyGroup));
	EXPECT_EQ(12, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::ObjectLabelRefrenceList));
	EXPECT_EQ(13, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::ExternalObjectDefinition));
	EXPECT_EQ(12, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::ExternalReferenceNAME));
	EXPECT_EQ(9, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::ExternalObjectPointer));
	EXPECT_EQ(17, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::Animation));

	// Don't support proprietary objects for autoscaling
	EXPECT_EQ(0, clientUnderTest.test_wrapper_get_minimum_object_length(VirtualTerminalObjectType::ManufacturerDefined11));

	CANNetworkManager::CANNetwork.deactivate_control_function(vtPartner);
	CANNetworkManager::CANNetwork.deactivate_control_function(internalECU);
}

TEST_F(VirtualTerminalTest, FontRemapping)
{
	DerivedTestVTClient clientUnderTest(nullptr, nullptr);

	// Check some easy 50% scaling cases
	EXPECT_EQ(clientUnderTest.test_wrapper_remap_font_to_scale(VirtualTerminalClient::FontSize::Size128x128, 0.5f), VirtualTerminalClient::FontSize::Size64x64);
	EXPECT_EQ(clientUnderTest.test_wrapper_remap_font_to_scale(VirtualTerminalClient::FontSize::Size64x64, 0.5f), VirtualTerminalClient::FontSize::Size32x32);
	EXPECT_EQ(clientUnderTest.test_wrapper_remap_font_to_scale(VirtualTerminalClient::FontSize::Size32x32, 0.5f), VirtualTerminalClient::FontSize::Size16x16);
	EXPECT_EQ(clientUnderTest.test_wrapper_remap_font_to_scale(VirtualTerminalClient::FontSize::Size16x16, 0.5f), VirtualTerminalClient::FontSize::Size8x8);

	// Ensure the floor of font sizes is 6x8
	EXPECT_EQ(clientUnderTest.test_wrapper_remap_font_to_scale(VirtualTerminalClient::FontSize::Size16x16, 0.00005f), VirtualTerminalClient::FontSize::Size6x8);
	EXPECT_EQ(clientUnderTest.test_wrapper_remap_font_to_scale(VirtualTerminalClient::FontSize::Size6x8, 0.00005f), VirtualTerminalClient::FontSize::Size6x8);

	// Check 75% scaling
	EXPECT_EQ(clientUnderTest.test_wrapper_remap_font_to_scale(VirtualTerminalClient::FontSize::Size128x192, 0.75f), VirtualTerminalClient::FontSize::Size96x128);

	// Check some easy 200% scaling cases
	EXPECT_EQ(clientUnderTest.test_wrapper_remap_font_to_scale(VirtualTerminalClient::FontSize::Size8x8, 2.0f), VirtualTerminalClient::FontSize::Size16x16);
	EXPECT_EQ(clientUnderTest.test_wrapper_remap_font_to_scale(VirtualTerminalClient::FontSize::Size16x16, 2.0f), VirtualTerminalClient::FontSize::Size32x32);
	EXPECT_EQ(clientUnderTest.test_wrapper_remap_font_to_scale(VirtualTerminalClient::FontSize::Size32x32, 2.0f), VirtualTerminalClient::FontSize::Size64x64);
	EXPECT_EQ(clientUnderTest.test_wrapper_remap_font_to_scale(VirtualTerminalClient::FontSize::Size64x64, 2.0f), VirtualTerminalClient::FontSize::Size128x128);

	// Ensure the size is capped at 196x128
	EXPECT_EQ(clientUnderTest.test_wrapper_remap_font_to_scale(VirtualTerminalClient::FontSize::Size32x32, 800.0f), VirtualTerminalClient::FontSize::Size128x192);

	// Check some partial upscaling
	EXPECT_EQ(clientUnderTest.test_wrapper_remap_font_to_scale(VirtualTerminalClient::FontSize::Size16x16, 1.5f), VirtualTerminalClient::FontSize::Size16x24);

	// Set and test supported Fonts
	clientUnderTest.test_wrapper_set_supported_fonts(0x55, 0x55); // 0x55 = 01010101

	// Small fonts
	EXPECT_EQ(true, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size6x8));
	EXPECT_EQ(false, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size8x8));
	EXPECT_EQ(true, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size8x12));
	EXPECT_EQ(false, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size12x16));
	EXPECT_EQ(true, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size16x16));
	EXPECT_EQ(false, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size16x24));
	EXPECT_EQ(true, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size24x32));
	EXPECT_EQ(false, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size32x32));

	// Large fonts
	EXPECT_EQ(true, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size32x48));
	EXPECT_EQ(false, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size48x64));
	EXPECT_EQ(true, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size64x64));
	EXPECT_EQ(false, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size64x96));
	EXPECT_EQ(true, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size96x128));
	EXPECT_EQ(false, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size128x128));
	EXPECT_EQ(true, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size128x192));

	// Remapping to the available fonts
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size6x8, clientUnderTest.test_wrapper_get_font_or_next_smallest_font((VirtualTerminalClient::FontSize::Size6x8)));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size6x8, clientUnderTest.test_wrapper_get_font_or_next_smallest_font((VirtualTerminalClient::FontSize::Size8x8)));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size8x12, clientUnderTest.test_wrapper_get_font_or_next_smallest_font((VirtualTerminalClient::FontSize::Size8x12)));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size8x12, clientUnderTest.test_wrapper_get_font_or_next_smallest_font((VirtualTerminalClient::FontSize::Size12x16)));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size16x16, clientUnderTest.test_wrapper_get_font_or_next_smallest_font(VirtualTerminalClient::FontSize::Size16x16));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size16x16, clientUnderTest.test_wrapper_get_font_or_next_smallest_font(VirtualTerminalClient::FontSize::Size16x24));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size24x32, clientUnderTest.test_wrapper_get_font_or_next_smallest_font(VirtualTerminalClient::FontSize::Size24x32));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size24x32, clientUnderTest.test_wrapper_get_font_or_next_smallest_font(VirtualTerminalClient::FontSize::Size32x32));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size32x48, clientUnderTest.test_wrapper_get_font_or_next_smallest_font(VirtualTerminalClient::FontSize::Size32x48));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size32x48, clientUnderTest.test_wrapper_get_font_or_next_smallest_font(VirtualTerminalClient::FontSize::Size48x64));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size64x64, clientUnderTest.test_wrapper_get_font_or_next_smallest_font(VirtualTerminalClient::FontSize::Size64x64));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size64x64, clientUnderTest.test_wrapper_get_font_or_next_smallest_font(VirtualTerminalClient::FontSize::Size64x96));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size96x128, clientUnderTest.test_wrapper_get_font_or_next_smallest_font(VirtualTerminalClient::FontSize::Size96x128));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size96x128, clientUnderTest.test_wrapper_get_font_or_next_smallest_font(VirtualTerminalClient::FontSize::Size128x128));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size128x192, clientUnderTest.test_wrapper_get_font_or_next_smallest_font(VirtualTerminalClient::FontSize::Size128x192));

	clientUnderTest.test_wrapper_set_supported_fonts(0xAA, 0xAA); // 0xAA = 10101010
	// Small fonts
	EXPECT_EQ(false, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size6x8));
	EXPECT_EQ(true, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size8x8));
	EXPECT_EQ(false, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size8x12));
	EXPECT_EQ(true, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size12x16));
	EXPECT_EQ(false, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size16x16));
	EXPECT_EQ(true, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size16x24));
	EXPECT_EQ(false, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size24x32));
	EXPECT_EQ(true, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size32x32));

	// Large fonts
	EXPECT_EQ(false, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size32x48));
	EXPECT_EQ(true, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size48x64));
	EXPECT_EQ(false, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size64x64));
	EXPECT_EQ(true, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size64x96));
	EXPECT_EQ(false, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size96x128));
	EXPECT_EQ(true, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size128x128));
	EXPECT_EQ(false, clientUnderTest.get_font_size_supported(VirtualTerminalClient::FontSize::Size128x192));

	// Remapping to the available fonts
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size6x8, clientUnderTest.test_wrapper_get_font_or_next_smallest_font((VirtualTerminalClient::FontSize::Size6x8)));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size8x8, clientUnderTest.test_wrapper_get_font_or_next_smallest_font((VirtualTerminalClient::FontSize::Size8x8)));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size8x8, clientUnderTest.test_wrapper_get_font_or_next_smallest_font((VirtualTerminalClient::FontSize::Size8x12)));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size12x16, clientUnderTest.test_wrapper_get_font_or_next_smallest_font((VirtualTerminalClient::FontSize::Size12x16)));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size12x16, clientUnderTest.test_wrapper_get_font_or_next_smallest_font(VirtualTerminalClient::FontSize::Size16x16));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size16x24, clientUnderTest.test_wrapper_get_font_or_next_smallest_font(VirtualTerminalClient::FontSize::Size16x24));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size16x24, clientUnderTest.test_wrapper_get_font_or_next_smallest_font(VirtualTerminalClient::FontSize::Size24x32));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size32x32, clientUnderTest.test_wrapper_get_font_or_next_smallest_font(VirtualTerminalClient::FontSize::Size32x32));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size32x32, clientUnderTest.test_wrapper_get_font_or_next_smallest_font(VirtualTerminalClient::FontSize::Size32x48));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size48x64, clientUnderTest.test_wrapper_get_font_or_next_smallest_font(VirtualTerminalClient::FontSize::Size48x64));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size48x64, clientUnderTest.test_wrapper_get_font_or_next_smallest_font(VirtualTerminalClient::FontSize::Size64x64));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size64x96, clientUnderTest.test_wrapper_get_font_or_next_smallest_font(VirtualTerminalClient::FontSize::Size64x96));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size64x96, clientUnderTest.test_wrapper_get_font_or_next_smallest_font(VirtualTerminalClient::FontSize::Size96x128));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size128x128, clientUnderTest.test_wrapper_get_font_or_next_smallest_font(VirtualTerminalClient::FontSize::Size128x128));
	EXPECT_EQ(VirtualTerminalClient::FontSize::Size128x128, clientUnderTest.test_wrapper_get_font_or_next_smallest_font(VirtualTerminalClient::FontSize::Size128x192));

	// It doesn't really make sense to test the hardcoded scales against the same arbitrary boundaries I made up, so just loop through all remappings.
	// If we discover good scale factors from real testing we can add them here instead.
	for (std::uint8_t i = 0; i <= static_cast<std::uint8_t>(VirtualTerminalClient::FontSize::Size128x192); i++)
	{
		for (float j = 0.0f; j <= 24.0f; j += 0.05f)
		{
			clientUnderTest.test_wrapper_remap_font_to_scale(static_cast<VirtualTerminalClient::FontSize>(i), j);
		}
	}
}

TEST_F(VirtualTerminalTest, ResizeOutputArchedBarGraph)
{
	constexpr std::uint16_t testWidth = 200;
	constexpr std::uint16_t testHeight = 100;
	std::uint8_t testObject[] = {
		0x00,
		0x01,
		0x13,
		testWidth & 0xFF,
		(testWidth >> 8),
		testHeight & 0xFF,
		(testHeight >> 8),
		0x07,
		0x00,
		0x03,
		0x00,
		0xB4,
		0x30,
		0x00,
		0x00,
		0x00,
		0xFF,
		0x00,
		0xFF,
		0xFF,
		0x00,
		0x00,
		0xFF,
		0xFF,
		0x10,
		0x00,
		0x00
	};

	DerivedTestVTClient clientUnderTest(nullptr, nullptr);

	EXPECT_EQ(true, clientUnderTest.test_wrapper_resize_object(testObject, 0.5f, VirtualTerminalObjectType::OutputArchedBarGraph));
	EXPECT_EQ(testWidth / 2, static_cast<std::uint16_t>(testObject[3]) | (static_cast<std::uint16_t>(testObject[4]) << 8));
	EXPECT_EQ(testHeight / 2, static_cast<std::uint16_t>(testObject[5]) | (static_cast<std::uint16_t>(testObject[6]) << 8));

	EXPECT_EQ(true, clientUnderTest.test_wrapper_resize_object(testObject, 2.0f, VirtualTerminalObjectType::OutputArchedBarGraph));
	EXPECT_EQ(testWidth, static_cast<std::uint16_t>(testObject[3]) | (static_cast<std::uint16_t>(testObject[4]) << 8));
	EXPECT_EQ(testHeight, static_cast<std::uint16_t>(testObject[5]) | (static_cast<std::uint16_t>(testObject[6]) << 8));
}

TEST_F(VirtualTerminalTest, ResizeOutputLinearBarGraph)
{
	constexpr std::uint16_t testWidth = 200;
	constexpr std::uint16_t testHeight = 100;
	std::uint8_t testObject[] = {
		0x00,
		0x01,
		0x12,
		testWidth & 0xFF,
		(testWidth >> 8),
		testHeight & 0xFF,
		(testHeight >> 8),
		0x07,
		0x00,
		0x03,
		0x00,
		0xB4,
		0x30,
		0x00,
		0x00,
		0x00,
		0xFF,
		0x00,
		0xFF,
		0xFF,
		0x00,
		0x00,
		0xFF,
		0x00
	};

	DerivedTestVTClient clientUnderTest(nullptr, nullptr);

	EXPECT_EQ(true, clientUnderTest.test_wrapper_resize_object(testObject, 0.5f, VirtualTerminalObjectType::OutputLinearBarGraph));
	EXPECT_EQ(testWidth / 2, static_cast<std::uint16_t>(testObject[3]) | (static_cast<std::uint16_t>(testObject[4]) << 8));
	EXPECT_EQ(testHeight / 2, static_cast<std::uint16_t>(testObject[5]) | (static_cast<std::uint16_t>(testObject[6]) << 8));

	EXPECT_EQ(true, clientUnderTest.test_wrapper_resize_object(testObject, 2.0f, VirtualTerminalObjectType::OutputLinearBarGraph));
	EXPECT_EQ(testWidth, static_cast<std::uint16_t>(testObject[3]) | (static_cast<std::uint16_t>(testObject[4]) << 8));
	EXPECT_EQ(testHeight, static_cast<std::uint16_t>(testObject[5]) | (static_cast<std::uint16_t>(testObject[6]) << 8));
}

TEST_F(VirtualTerminalTest, ResizeOutputMeter)
{
	constexpr std::uint16_t testWidth = 200;
	std::uint8_t testObject[] = {
		0x00,
		0x01,
		0x11,
		testWidth & 0xFF,
		(testWidth >> 8),
		0x00,
		0x00,
		0x07,
		0x00,
		0x03,
		0x00,
		0xB4,
		0x30,
		0x00,
		0x00,
		0x00,
		0xFF,
		0x00,
		0xFF,
		0xFF,
		0x00
	};

	DerivedTestVTClient clientUnderTest(nullptr, nullptr);

	EXPECT_EQ(true, clientUnderTest.test_wrapper_resize_object(testObject, 0.5f, VirtualTerminalObjectType::OutputMeter));
	EXPECT_EQ(testWidth / 2, static_cast<std::uint16_t>(testObject[3]) | (static_cast<std::uint16_t>(testObject[4]) << 8));

	EXPECT_EQ(true, clientUnderTest.test_wrapper_resize_object(testObject, 2.0f, VirtualTerminalObjectType::OutputMeter));
	EXPECT_EQ(testWidth, static_cast<std::uint16_t>(testObject[3]) | (static_cast<std::uint16_t>(testObject[4]) << 8));
}

TEST_F(VirtualTerminalTest, ResizeOutputPolygon)
{
	constexpr std::uint16_t testWidth = 200;
	constexpr std::uint16_t testHeight = 100;
	std::uint8_t testObject[] = {
		0x00,
		0x01,
		0x10,
		testWidth & 0xFF,
		(testWidth >> 8),
		testHeight & 0xFF,
		(testHeight >> 8),
		0xFF,
		0xFF,
		0xFF,
		0xFF,
		0xFF,
		0x00,
		0x00
	};

	DerivedTestVTClient clientUnderTest(nullptr, nullptr);

	EXPECT_EQ(true, clientUnderTest.test_wrapper_resize_object(testObject, 0.5f, VirtualTerminalObjectType::OutputPolygon));
	EXPECT_EQ(testWidth / 2, static_cast<std::uint16_t>(testObject[3]) | (static_cast<std::uint16_t>(testObject[4]) << 8));
	EXPECT_EQ(testHeight / 2, static_cast<std::uint16_t>(testObject[5]) | (static_cast<std::uint16_t>(testObject[6]) << 8));

	EXPECT_EQ(true, clientUnderTest.test_wrapper_resize_object(testObject, 2.0f, VirtualTerminalObjectType::OutputPolygon));
	EXPECT_EQ(testWidth, static_cast<std::uint16_t>(testObject[3]) | (static_cast<std::uint16_t>(testObject[4]) << 8));
	EXPECT_EQ(testHeight, static_cast<std::uint16_t>(testObject[5]) | (static_cast<std::uint16_t>(testObject[6]) << 8));
}

TEST_F(VirtualTerminalTest, ResizeOutputEllipse)
{
	constexpr std::uint16_t testWidth = 200;
	constexpr std::uint16_t testHeight = 100;
	std::uint8_t testObject[] = {
		0x00,
		0x01,
		0x0F,
		0xFF,
		0xFF,
		testWidth & 0xFF,
		(testWidth >> 8),
		testHeight & 0xFF,
		(testHeight >> 8),
		0x00,
		0x00,
		0xFF,
		0xFF,
		0xFF,
		0x00
	};

	DerivedTestVTClient clientUnderTest(nullptr, nullptr);

	EXPECT_EQ(true, clientUnderTest.test_wrapper_resize_object(testObject, 0.5f, VirtualTerminalObjectType::OutputEllipse));
	EXPECT_EQ(testWidth / 2, static_cast<std::uint16_t>(testObject[5]) | (static_cast<std::uint16_t>(testObject[6]) << 8));
	EXPECT_EQ(testHeight / 2, static_cast<std::uint16_t>(testObject[7]) | (static_cast<std::uint16_t>(testObject[8]) << 8));

	EXPECT_EQ(true, clientUnderTest.test_wrapper_resize_object(testObject, 2.0f, VirtualTerminalObjectType::OutputEllipse));
	EXPECT_EQ(testWidth, static_cast<std::uint16_t>(testObject[5]) | (static_cast<std::uint16_t>(testObject[6]) << 8));
	EXPECT_EQ(testHeight, static_cast<std::uint16_t>(testObject[7]) | (static_cast<std::uint16_t>(testObject[8]) << 8));
}

TEST_F(VirtualTerminalTest, ResizeOutputLine)
{
	constexpr std::uint16_t testWidth = 200;
	constexpr std::uint16_t testHeight = 100;
	std::uint8_t testObject[] = {
		0x00,
		0x01,
		0x0D,
		0xFF,
		0xFF,
		testWidth & 0xFF,
		(testWidth >> 8),
		testHeight & 0xFF,
		(testHeight >> 8),
		0xFF,
		0xFF
	};

	DerivedTestVTClient clientUnderTest(nullptr, nullptr);

	EXPECT_EQ(true, clientUnderTest.test_wrapper_resize_object(testObject, 0.5f, VirtualTerminalObjectType::OutputLine));
	EXPECT_EQ(testWidth / 2, static_cast<std::uint16_t>(testObject[5]) | (static_cast<std::uint16_t>(testObject[6]) << 8));
	EXPECT_EQ(testHeight / 2, static_cast<std::uint16_t>(testObject[7]) | (static_cast<std::uint16_t>(testObject[8]) << 8));

	EXPECT_EQ(true, clientUnderTest.test_wrapper_resize_object(testObject, 2.0f, VirtualTerminalObjectType::OutputLine));
	EXPECT_EQ(testWidth, static_cast<std::uint16_t>(testObject[5]) | (static_cast<std::uint16_t>(testObject[6]) << 8));
	EXPECT_EQ(testHeight, static_cast<std::uint16_t>(testObject[7]) | (static_cast<std::uint16_t>(testObject[8]) << 8));
}

TEST_F(VirtualTerminalTest, ResizeOutputList)
{
	constexpr std::uint16_t testWidth = 200;
	constexpr std::uint16_t testHeight = 100;
	std::uint8_t testObject[] = {
		0x00,
		0x01,
		0x25,
		testWidth & 0xFF,
		(testWidth >> 8),
		testHeight & 0xFF,
		(testHeight >> 8),
		0xFF,
		0xFF,
		0xFF,
		0x00,
		0x00,
		0x00,
		0x00
	};

	DerivedTestVTClient clientUnderTest(nullptr, nullptr);

	// Check object length
	EXPECT_EQ(12, clientUnderTest.test_wrapper_get_number_bytes_in_object(testObject));

	// Add a macro and re-check the length
	testObject[11] = 1;
	EXPECT_EQ(14, clientUnderTest.test_wrapper_get_number_bytes_in_object(testObject));

	// Add a full list of child objects and re-check the length
	testObject[10] = 255;
	EXPECT_EQ(524, clientUnderTest.test_wrapper_get_number_bytes_in_object(testObject));

	EXPECT_EQ(true, clientUnderTest.test_wrapper_resize_object(testObject, 0.5f, VirtualTerminalObjectType::OutputList));
	EXPECT_EQ(testWidth / 2, static_cast<std::uint16_t>(testObject[3]) | (static_cast<std::uint16_t>(testObject[4]) << 8));
	EXPECT_EQ(testHeight / 2, static_cast<std::uint16_t>(testObject[5]) | (static_cast<std::uint16_t>(testObject[6]) << 8));

	EXPECT_EQ(true, clientUnderTest.test_wrapper_resize_object(testObject, 2.0f, VirtualTerminalObjectType::OutputList));
	EXPECT_EQ(testWidth, static_cast<std::uint16_t>(testObject[3]) | (static_cast<std::uint16_t>(testObject[4]) << 8));
	EXPECT_EQ(testHeight, static_cast<std::uint16_t>(testObject[5]) | (static_cast<std::uint16_t>(testObject[6]) << 8));
}

TEST_F(VirtualTerminalTest, ResizeInputBoolean)
{
	constexpr std::uint16_t testWidth = 50;
	std::uint8_t testObject[] = {
		0x00,
		0x01,
		0x07,
		0x00,
		testWidth & 0xFF,
		(testWidth >> 8),
		0xFF,
		0xFF,
		0xFF,
		0xFF,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00
	};

	DerivedTestVTClient clientUnderTest(nullptr, nullptr);

	// Check object length
	EXPECT_EQ(13, clientUnderTest.test_wrapper_get_number_bytes_in_object(testObject));

	// Add a macro and re-check the length
	testObject[12] = 1;
	EXPECT_EQ(15, clientUnderTest.test_wrapper_get_number_bytes_in_object(testObject));

	// Cant really resize these since the width is a max value. Should remain the same.
	EXPECT_EQ(true, clientUnderTest.test_wrapper_resize_object(testObject, 2.0f, VirtualTerminalObjectType::InputBoolean));
	EXPECT_EQ(testWidth, static_cast<std::uint16_t>(testObject[4]) | (static_cast<std::uint16_t>(testObject[5]) << 8));
}

TEST_F(VirtualTerminalTest, TestNumberBytesInInvalidObjects)
{
	DerivedTestVTClient clientUnderTest(nullptr, nullptr);

	// Test some unsupported objects
	for (auto i = static_cast<std::uint8_t>(VirtualTerminalObjectType::ManufacturerDefined1); i < static_cast<std::uint8_t>(VirtualTerminalObjectType::Reserved); i++)
	{
		std::uint8_t testObject[] = {
			0x00,
			0x01,
			i
		};
		EXPECT_EQ(0, clientUnderTest.test_wrapper_get_number_bytes_in_object(testObject));
	}
}

TEST_F(VirtualTerminalTest, MessageConstruction)
{
	VirtualCANPlugin serverVT;
	serverVT.open();

	CANHardwareInterface::set_number_of_can_channels(1);
	CANHardwareInterface::assign_can_channel_frame_handler(0, std::make_shared<VirtualCANPlugin>());
	CANHardwareInterface::start(false);

	auto internalECU = test_helpers::claim_internal_control_function(0x37, 0, time_source);
	auto vtPartner = test_helpers::force_claim_partnered_control_function(0x26, 0);

	DerivedTestVTClient interfaceUnderTest(vtPartner, internalECU);
	interfaceUnderTest.initialize(false);

	time_source.update_for_ms(50);

	// Get the virtual CAN plugin back to a known state
	CANMessageFrame testFrame = {};
	while (!serverVT.get_queue_empty())
	{
		serverVT.read_frame(testFrame);
	}
	ASSERT_TRUE(serverVT.get_queue_empty());

	// Test send change active mask command while not connected queues the command
	ASSERT_TRUE(interfaceUnderTest.send_change_active_mask(123, 456));
	ASSERT_TRUE(serverVT.get_queue_empty());
	interfaceUnderTest.test_wrapper_set_state(VirtualTerminalClient::StateMachineState::Connected);
	interfaceUnderTest.test_wrapper_process_command_queue();
	time_source.update_for_ms(5);

	ASSERT_TRUE(serverVT.read_frame(testFrame));
	EXPECT_EQ(0, testFrame.channel);
	EXPECT_EQ(CAN_DATA_LENGTH, testFrame.dataLength);
	EXPECT_TRUE(testFrame.isExtendedFrame);
	EXPECT_EQ(0x14E72637, testFrame.identifier);

	EXPECT_EQ(173, testFrame.data[0]); // VT Function

	std::uint16_t workingSetObjectID = (static_cast<std::uint16_t>(testFrame.data[1]) | (static_cast<std::uint16_t>(testFrame.data[2]) << 8));
	EXPECT_EQ(123, workingSetObjectID);

	std::uint16_t newActiveMaskObjectID = (static_cast<std::uint16_t>(testFrame.data[3]) | (static_cast<std::uint16_t>(testFrame.data[4]) << 8));
	EXPECT_EQ(456, newActiveMaskObjectID);

	// Test send_hide_show_object, but since we have not yet sent a response to the change active mask command, it should queue the command
	ASSERT_TRUE(interfaceUnderTest.send_hide_show_object(1234, VirtualTerminalClient::HideShowObjectCommand::HideObject));
	ASSERT_TRUE(serverVT.get_queue_empty());
	interfaceUnderTest.test_wrapper_process_command_queue();
	time_source.update_for_ms(5);
	ASSERT_FALSE(serverVT.read_frame(testFrame));

	// Send a response to the change active mask command
	testFrame.identifier = 0x14E63726; // VT->ECU
	testFrame.data[0] = 173; // VT Function
	testFrame.data[1] = 123 & 0xFF;
	testFrame.data[2] = (123 >> 8) & 0xFF;
	testFrame.data[3] = 0; // Success
	testFrame.data[4] = 0xFF; // Reserved
	testFrame.data[5] = 0xFF; // Reserved
	testFrame.data[6] = 0xFF; // Reserved
	testFrame.data[7] = 0xFF; // Reserved
	CANNetworkManager::CANNetwork.process_receive_can_message_frame(testFrame);
	CANNetworkManager::CANNetwork.update();

	interfaceUnderTest.test_wrapper_process_command_queue();
	time_source.update_for_ms(5);

	ASSERT_TRUE(serverVT.read_frame(testFrame));
	EXPECT_EQ(0, testFrame.channel);
	EXPECT_EQ(CAN_DATA_LENGTH, testFrame.dataLength);
	EXPECT_TRUE(testFrame.isExtendedFrame);
	EXPECT_EQ(0x14E72637, testFrame.identifier);
	EXPECT_EQ(160, testFrame.data[0]); // VT function

	std::uint16_t objectID = (static_cast<std::uint16_t>(testFrame.data[1]) | (static_cast<std::uint16_t>(testFrame.data[2]) << 8));
	EXPECT_EQ(1234, objectID);
	EXPECT_EQ(0, testFrame.data[3]); // Hide
	EXPECT_EQ(0xFF, testFrame.data[4]); // Reserved
	EXPECT_EQ(0xFF, testFrame.data[5]); // Reserved
	EXPECT_EQ(0xFF, testFrame.data[6]); // Reserved
	EXPECT_EQ(0xFF, testFrame.data[7]); // Reserved

	// Send a response to the hide object command
	testFrame.identifier = 0x14E63726; // VT->ECU
	testFrame.data[0] = 160; // VT Function
	testFrame.data[1] = 1234 & 0xFF;
	testFrame.data[2] = (1234 >> 8) & 0xFF;
	testFrame.data[3] = 0; // Hide
	testFrame.data[4] = 0xFF; // Reserved
	testFrame.data[5] = 0xFF; // Reserved
	testFrame.data[6] = 0xFF; // Reserved
	testFrame.data[7] = 0xFF; // Reserved
	CANNetworkManager::CANNetwork.process_receive_can_message_frame(testFrame);
	CANNetworkManager::CANNetwork.update();

	ASSERT_TRUE(serverVT.get_queue_empty());
	ASSERT_TRUE(interfaceUnderTest.send_enable_disable_object(1234, VirtualTerminalClient::EnableDisableObjectCommand::DisableObject));
	time_source.update_for_ms(5);
	ASSERT_TRUE(serverVT.read_frame(testFrame));
	EXPECT_EQ(0, testFrame.channel);
	EXPECT_EQ(CAN_DATA_LENGTH, testFrame.dataLength);
	EXPECT_TRUE(testFrame.isExtendedFrame);
	EXPECT_EQ(0x14E72637, testFrame.identifier);
	EXPECT_EQ(161, testFrame.data[0]); // VT function
	objectID = (static_cast<std::uint16_t>(testFrame.data[1]) | (static_cast<std::uint16_t>(testFrame.data[2]) << 8));
	EXPECT_EQ(1234, objectID);
	EXPECT_EQ(0, testFrame.data[3]); // Disable

	// Send a response to the disable object command
	testFrame.identifier = 0x14E63726; // VT->ECU
	testFrame.data[0] = 161; // VT Function
	testFrame.data[1] = 1234 & 0xFF;
	testFrame.data[2] = (1234 >> 8) & 0xFF;
	testFrame.data[3] = 0; // Disable
	testFrame.data[4] = 0xFF; // Reserved
	testFrame.data[5] = 0xFF; // Reserved
	testFrame.data[6] = 0xFF; // Reserved
	testFrame.data[7] = 0xFF; // Reserved
	CANNetworkManager::CANNetwork.process_receive_can_message_frame(testFrame);
	CANNetworkManager::CANNetwork.update();

	// Test draw text
	const std::string testString = "a";
	ASSERT_TRUE(serverVT.get_queue_empty());
	ASSERT_TRUE(interfaceUnderTest.send_draw_text(123, true, 1, testString.data()));
	time_source.update_for_ms(5);
	ASSERT_TRUE(serverVT.read_frame(testFrame));
	EXPECT_EQ(0, testFrame.channel);
	EXPECT_EQ(CAN_DATA_LENGTH, testFrame.dataLength);
	EXPECT_TRUE(testFrame.isExtendedFrame);
	EXPECT_EQ(0x14E72637, testFrame.identifier);
	EXPECT_EQ(184, testFrame.data[0]); // VT function (graphics context command)
	objectID = (static_cast<std::uint16_t>(testFrame.data[1]) | (static_cast<std::uint16_t>(testFrame.data[2]) << 8));
	EXPECT_EQ(123, objectID);
	EXPECT_EQ(1, testFrame.data[4]); // Transparent
	EXPECT_EQ(1, testFrame.data[5]); // Length
	EXPECT_EQ('a', testFrame.data[6]);

	serverVT.close();
	CANHardwareInterface::stop();

	CANNetworkManager::CANNetwork.deactivate_control_function(vtPartner);
	CANNetworkManager::CANNetwork.deactivate_control_function(internalECU);
}
