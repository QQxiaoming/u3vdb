#ifdef _WIN32
#include "libusb.h"
#else
#include <libusb-1.0/libusb.h>
#endif

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <functional>

#ifdef _WIN32
	#include <windows.h>
	#include <conio.h>
	#include <io.h>
	#ifndef STDIN_FILENO
		#define STDIN_FILENO 0
	#endif
	#ifndef STDOUT_FILENO
		#define STDOUT_FILENO 1
	#endif
#else
	#include <termios.h>
	#include <unistd.h>
	#include <sys/select.h>
#endif

namespace {

#define TY_UVCP_MAX_MSG_LEN 65536

//UVCP (USB3 Vision Control Protocol)
namespace UVCPConstants {
    const uint32_t MAGIC = 0x43563355; // "U3VC"
    const uint16_t FLAGS_REQUEST_ACK = 0x0001<<14;
    
    enum Command {
        COMMAND_READ_MEMORY_CMD                 = 0x0800,
        COMMAND_READ_MEMORY_ACK                 = 0x0801,
        COMMAND_WRITE_MEMORY_CMD                = 0x0802,
        COMMAND_WRITE_MEMORY_ACK                = 0x0803,
        COMMAND_PENDING_ACK                     = 0x0805,
        COMMAND_EVENT_CMD                       = 0x0c00,
	    COMMAND_EVENT_ACK                       = 0x0c01
    };
}

#pragma pack(push, 1)
struct ManifestEntry {
    uint16_t file_version_subminor;
    uint8_t file_version_minor;
    uint8_t file_version_major;
    uint32_t schema;
    uint64_t address;
    uint64_t size;
    uint64_t unknown3;
    uint64_t unknown4;
    uint64_t unknown5;
    uint64_t unknown6;
    uint64_t unknown7;
};

struct UVCPHeader { //ArvUvcpHeader
    uint32_t magic  = 0;
    uint16_t flags  = 0;
    uint16_t command= 0;
    uint16_t size   = 0;
    uint16_t id     = 0;
};

struct UVCPReadMemoryCmd {  //ArvUvcpReadMemoryCmd
    UVCPHeader header{};
    uint64_t address = 0;
    uint16_t unknown = 0;
    uint16_t size    = 0;
};

struct UVCPReadMemoryAck {      //ArvUvcpReadMemoryCmd
    UVCPHeader header{};
    uint8_t data[1]; // 可变长度数据
};

struct UVCPWriteMemoryCmd {      //ArvUvcpWriteMemoryCmd
    UVCPHeader header{};
    uint64_t address = 0;
    uint8_t data[1]; // 可变长度数据
};

struct UVCPWriteMemoryAck {     //ArvUvcpWriteMemoryAck
    UVCPHeader header{};
    uint16_t unknown       = 0;
    uint16_t bytes_written = 0;
};

struct UVCPPendingAck {
    UVCPHeader header{};
    uint16_t unknown    = 0;
    uint16_t timeout_ms = 0;
};
#pragma pack(pop)

constexpr uint16_t kVendorId = 0x04b4;
constexpr uint16_t kProductId = 0x1003;
// Control interface and endpoints are discovered dynamically.
constexpr unsigned int kTransferTimeoutMs = 10000;

constexpr uint32_t kTerminalMagic = 0x5445524Du; // 'TERM'
constexpr uint32_t kTerminalBaseAddr       = 0x30000;
constexpr uint32_t kTerminalVersionAddr    = kTerminalBaseAddr + 0x4;
constexpr uint32_t kTerminalStatusAddr     = kTerminalBaseAddr + 0x8;
constexpr uint32_t kTerminalAvailAddr      = kTerminalBaseAddr + 0xC;
constexpr uint32_t kTerminalChunkHintAddr  = kTerminalBaseAddr + 0x10;
constexpr uint32_t kTerminalAuthStatusAddr = kTerminalBaseAddr + 0x14;
constexpr uint32_t kTerminalAuthCmdAddr    = kTerminalBaseAddr + 0x18;
constexpr uint32_t kTerminalAuthBufAddr    = kTerminalBaseAddr + 0x1C;
constexpr uint32_t kTerminalDataAddr       = kTerminalBaseAddr + 0x100;

constexpr uint32_t kTerminalFileCmdAddr         = kTerminalBaseAddr + 0x40;
constexpr uint32_t kTerminalFileStatusAddr      = kTerminalBaseAddr + 0x44;
constexpr uint32_t kTerminalFileResultAddr      = kTerminalBaseAddr + 0x48;
constexpr uint32_t kTerminalFileSizeLowAddr     = kTerminalBaseAddr + 0x4C;
constexpr uint32_t kTerminalFileSizeHighAddr    = kTerminalBaseAddr + 0x50;
constexpr uint32_t kTerminalFileCursorLowAddr   = kTerminalBaseAddr + 0x54;
constexpr uint32_t kTerminalFileCursorHighAddr  = kTerminalBaseAddr + 0x58;
constexpr uint32_t kTerminalFileDataAvailAddr   = kTerminalBaseAddr + 0x5C;
constexpr uint32_t kTerminalFilePathAddr        = kTerminalBaseAddr + 0x60;
constexpr uint32_t kTerminalFilePathCapacity    = 0x60;
constexpr uint32_t kTerminalFileDataAddr        = kTerminalBaseAddr + 0xC0;
constexpr uint32_t kTerminalFileDataWindow      = 0x40;

constexpr uint32_t kStatusReady = 1u << 0;
constexpr uint32_t kStatusChildAlive = 1u << 1;
constexpr uint32_t kStatusOutputPending = 1u << 2;
constexpr uint32_t kStatusOverflow = 1u << 3;
constexpr uint32_t kStatusError = 1u << 4;

constexpr uint32_t kCtrlStart = 1u << 0;
constexpr uint32_t kCtrlReset = 1u << 1;
constexpr uint32_t kCtrlSigInt = 1u << 2;
constexpr uint32_t kCtrlSigTerm = 1u << 3;
constexpr uint32_t kCtrlClearFlags = 1u << 4;
constexpr uint32_t kCtrlEchoEnable = 1u << 5;
constexpr uint32_t kCtrlEchoDisable = 1u << 6;

enum FileCommandReg : uint32_t {
	kFileCmdNone     = 0,
	kFileCmdOpenRead = 1,
	kFileCmdOpenWrite= 2,
	kFileCmdClose    = 3,
	kFileCmdReset    = 4,
};

constexpr uint32_t kFileStatusBusy      = 1u << 0;
constexpr uint32_t kFileStatusError     = 1u << 1;
constexpr uint32_t kFileStatusEof       = 1u << 2;
constexpr uint32_t kFileStatusReading   = 1u << 3;
constexpr uint32_t kFileStatusWriting   = 1u << 4;
constexpr uint32_t kFileStatusOpen      = 1u << 5;
constexpr uint32_t kFileStatusPathReady = 1u << 6;

class U3VDevice {
  public:
	U3VDevice() = default;
	~U3VDevice() { shutdown(); }

	bool open(uint16_t vendorId, uint16_t productId, const std::string& serialFilter = {}) {
		if (ctx_) {
			std::cerr << "Context already initialized" << std::endl;
			return false;
		}

		if (int err = libusb_init(&ctx_); err != LIBUSB_SUCCESS) {
			std::cerr << "libusb_init failed: " << libusb_error_name(err) << std::endl;
			ctx_ = nullptr;
			return false;
		}

		libusb_device** list = nullptr;
		ssize_t count = libusb_get_device_list(ctx_, &list);
		if (count < 0 || !list) {
			std::cerr << "libusb_get_device_list failed" << std::endl;
			libusb_exit(ctx_);
			ctx_ = nullptr;
			return false;
		}

		struct DeviceCandidate {
			libusb_device_handle* handle = nullptr;
			std::string serial;
		};
		bool found = false;
		std::vector<DeviceCandidate> candidates;
		for (ssize_t i = 0; i < count; ++i) {
			libusb_device* dev = list[i];
			libusb_device_descriptor desc{};
			if (libusb_get_device_descriptor(dev, &desc) != LIBUSB_SUCCESS) {
				continue;
			}
			if (desc.idVendor != vendorId || desc.idProduct != productId) {
				continue;
			}

			libusb_device_handle* candidate = nullptr;
			if (libusb_open(dev, &candidate) != LIBUSB_SUCCESS || !candidate) {
				continue;
			}

			std::string serial;
			if (desc.iSerialNumber != 0) {
				unsigned char buffer[256] = {0};
				int len = libusb_get_string_descriptor_ascii(candidate, desc.iSerialNumber, buffer,
													 sizeof(buffer));
				if (len > 0) {
					serial.assign(reinterpret_cast<char*>(buffer), len);
				}
			}

			if (!serialFilter.empty()) {
				if (!serial.empty() && serial == serialFilter) {
					handle_ = candidate;
					found = true;
					break;
				}
				libusb_close(candidate);
				continue;
			}

			candidates.push_back(DeviceCandidate{candidate, serial});
		}

		libusb_free_device_list(list, 1);
		if (!serialFilter.empty()) {
			if (!found) {
				std::cerr << "Unable to open device " << std::hex << vendorId << ':' << productId
					  << " with serial '" << serialFilter << '\'' << std::dec << std::endl;
				libusb_exit(ctx_);
				ctx_ = nullptr;
				return false;
			}
		} else {
			if (candidates.empty()) {
				std::cerr << "Unable to open device " << std::hex << vendorId << ':' << productId
					  << std::dec << std::endl;
				libusb_exit(ctx_);
				ctx_ = nullptr;
				return false;
			}
			if (candidates.size() == 1) {
				handle_ = candidates[0].handle;
				found = true;
			} else {
				std::cout << "Multiple USB3 Vision devices detected:" << std::endl;
				for (size_t idx = 0; idx < candidates.size(); ++idx) {
					libusb_device* dev = libusb_get_device(candidates[idx].handle);
					uint8_t bus = libusb_get_bus_number(dev);
					uint8_t addr = libusb_get_device_address(dev);
					const std::string serial = candidates[idx].serial.empty()
												? std::string("<no-serial>")
												: candidates[idx].serial;
					std::cout << "  [" << idx << "] bus " << static_cast<int>(bus)
						  << " addr " << static_cast<int>(addr)
						  << ", serial: " << serial << std::endl;
				}
				std::cout << "Select device index: " << std::flush;
				std::string line;
				bool validChoice = false;
				while (std::getline(std::cin, line)) {
					if (line.empty()) {
						std::cout << "Select device index: " << std::flush;
						continue;
					}
					std::istringstream iss(line);
					size_t idx = 0;
					if (iss >> idx && idx < candidates.size()) {
						handle_ = candidates[idx].handle;
						validChoice = true;
						break;
					}
					std::cout << "Invalid selection. Enter a number between 0 and "
						  << (candidates.size() - 1) << ": " << std::flush;
				}
				if (!validChoice) {
					for (auto& c : candidates) {
						libusb_close(c.handle);
					}
					std::cerr << "Failed to select device" << std::endl;
					libusb_exit(ctx_);
					ctx_ = nullptr;
					return false;
				}
			}
			for (auto& c : candidates) {
				if (c.handle != handle_) {
					libusb_close(c.handle);
				}
			}
			found = handle_ != nullptr;
		}

		if (!found) {
			std::cerr << "Unable to open device " << std::hex << vendorId << ':' << productId;
			if (!serialFilter.empty()) {
				std::cerr << " with serial '" << serialFilter << '\'';
			}
			std::cerr << std::dec << std::endl;
			libusb_exit(ctx_);
			ctx_ = nullptr;
			return false;
		}

		std::cout << "Opened USB3 Vision device " << std::hex << vendorId << ':' << productId;
		if (!serialFilter.empty()) {
			std::cout << " (serial=" << serialFilter << ')';
		}
		std::cout << std::dec << std::endl;
		return true;
	}

	bool claimInterface(uint8_t interfaceNumber, uint8_t epOut, uint8_t epIn) {
		if (!handle_) {
			std::cerr << "Device handle is null" << std::endl;
			return false;
		}

		interfaceNumber_ = interfaceNumber;
		bulkOut_ = epOut;
		bulkIn_ = epIn;

		if (libusb_kernel_driver_active(handle_, interfaceNumber_) == 1) {
			const int detach = libusb_detach_kernel_driver(handle_, interfaceNumber_);
			if (detach != LIBUSB_SUCCESS) {
				std::cerr << "Failed to detach kernel driver: " << libusb_error_name(detach)
						  << std::endl;
				return false;
			}
		}

		const int claim = libusb_claim_interface(handle_, interfaceNumber_);
		if (claim != LIBUSB_SUCCESS) {
			std::cerr << "Failed to claim interface " << static_cast<int>(interfaceNumber_)
					  << ": " << libusb_error_name(claim) << std::endl;
			return false;
		}

		std::cout << "Claimed interface " << static_cast<int>(interfaceNumber_)
				  << " (OUT=0x" << std::hex << static_cast<int>(bulkOut_)
				  << ", IN=0x" << static_cast<int>(bulkIn_) << ")" << std::dec << std::endl;
		claimed_ = true;
		return true;
	}

	// Discover the USB3 Vision (U3V) control interface and bulk IN/OUT endpoints.
	// Matches interfaces with Class=0xEF (Misc), SubClass=0x05 (USB3 Vision), Protocol=0.
	bool findU3VControlInterface(uint8_t& outInterface, uint8_t& outEpOut, uint8_t& outEpIn) {
		if (!handle_) {
			std::cerr << "Device handle is null" << std::endl;
			return false;
		}

		libusb_device* dev = libusb_get_device(handle_);
		if (!dev) {
			std::cerr << "Failed to get libusb_device from handle" << std::endl;
			return false;
		}

		libusb_config_descriptor* cfg = nullptr;
		int rc = libusb_get_active_config_descriptor(dev, &cfg);
		if (rc != LIBUSB_SUCCESS || !cfg) {
			std::cerr << "libusb_get_active_config_descriptor failed: "
					  << libusb_error_name(rc) << std::endl;
			return false;
		}

		bool found = false;
		for (int i = 0; i < cfg->bNumInterfaces && !found; ++i) {
			const libusb_interface& intf = cfg->interface[i];
			for (int a = 0; a < intf.num_altsetting && !found; ++a) {
				const libusb_interface_descriptor& idesc = intf.altsetting[a];
				if (idesc.bInterfaceClass == 0xEF && // Miscellaneous Device
					idesc.bInterfaceSubClass == 0x05 && // USB3 Vision
					idesc.bInterfaceProtocol == 0x00) {
					uint8_t epIn = 0, epOut = 0;
					for (int e = 0; e < idesc.bNumEndpoints; ++e) {
						const libusb_endpoint_descriptor& ep = idesc.endpoint[e];
						uint8_t type = ep.bmAttributes & 0x3; // transfer type mask
						if (type == LIBUSB_TRANSFER_TYPE_BULK) {
							if (ep.bEndpointAddress & 0x80) {
								epIn = ep.bEndpointAddress;
							} else {
								epOut = ep.bEndpointAddress;
							}
						}
					}
					if (epIn != 0 && epOut != 0) {
						outInterface = idesc.bInterfaceNumber;
						outEpIn = epIn;
						outEpOut = epOut;
						found = true;
					}
				}
			}
		}

		libusb_free_config_descriptor(cfg);
		if (!found) {
			std::cerr << "No USB3 Vision control interface with bulk IN/OUT found" << std::endl;
		}
		return found;
	}

	bool readRegisters(uint32_t address, uint16_t registerCount, std::vector<uint32_t>& outValues) {
		if (registerCount == 0) {
			outValues.clear();
			return true;
		}

		const uint16_t bytesToRead = static_cast<uint16_t>(registerCount * 4);
		std::vector<uint8_t> raw;
		if (!readMemory(address, bytesToRead, raw)) {
			return false;
		}
		outValues.resize(registerCount);
		for (uint16_t i = 0; i < registerCount; ++i) {
			uint32_t v = 0;
			std::memcpy(&v, raw.data() + i * 4, 4);
			outValues[i] = v;
		}
		return true;
	}

	bool readMemory(uint32_t address, uint16_t bytes, std::vector<uint8_t>& outBytes) {
		if (!claimed_) {
			std::cerr << "Interface not claimed" << std::endl;
			return false;
		}
		if (bytes == 0) {
			outBytes.clear();
			return true;
		}

		UVCPReadMemoryCmd cmd{};
		cmd.header.magic = UVCPConstants::MAGIC;
		cmd.header.flags = UVCPConstants::FLAGS_REQUEST_ACK;
		cmd.header.command = UVCPConstants::COMMAND_READ_MEMORY_CMD;
		cmd.header.size = sizeof(cmd.address) + sizeof(cmd.unknown) + sizeof(cmd.size);
		cmd.header.id = nextRequestId();
		cmd.address = static_cast<uint64_t>(address);
		cmd.unknown = 0;
		cmd.size = bytes;

		if (!bulkSend(&cmd, sizeof(cmd))) {
			return false;
		}

		std::array<uint8_t, TY_UVCP_MAX_MSG_LEN> buffer{};
		int pendingLoops = 0;
		constexpr int kMaxPendingLoops = 5;
		while (true) {
			if (!bulkReceive(buffer.data(), static_cast<int>(buffer.size()))) {
				return false;
			}

			auto* hdr = reinterpret_cast<UVCPHeader*>(buffer.data());
			if (hdr->magic != UVCPConstants::MAGIC) {
				std::cerr << "Invalid ACK magic" << std::endl;
				return false;
			}
			if (hdr->id != cmd.header.id) {
				std::cerr << "ACK id mismatch: got " << hdr->id << ", expected " << cmd.header.id
						  << std::endl;
				return false;
			}

			if (hdr->command == UVCPConstants::COMMAND_PENDING_ACK) {
				auto* p = reinterpret_cast<UVCPPendingAck*>(buffer.data());
				const uint16_t waitMs = p->timeout_ms;
				if (++pendingLoops > kMaxPendingLoops) {
					std::cerr << "Too many PENDING_ACK responses" << std::endl;
					return false;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(waitMs ? waitMs : 1));
				continue;
			}

			if (hdr->command != UVCPConstants::COMMAND_READ_MEMORY_ACK) {
				std::cerr << "Unexpected ACK command: 0x" << std::hex << hdr->command << std::dec
						  << std::endl;
				return false;
			}

			if (hdr->size != bytes) {
				std::cerr << "Read size mismatch: got " << hdr->size << ", expected " << bytes
						  << std::endl;
				return false;
			}

			auto* ack = reinterpret_cast<UVCPReadMemoryAck*>(buffer.data());
			outBytes.assign(ack->data, ack->data + bytes);
			return true;
		}
	}

	bool writeRegister(uint32_t address, uint32_t value) {
		return writeRegisters(address, std::vector<uint32_t>{value});
	}

	bool writeRegisters(uint32_t startAddress, const std::vector<uint32_t>& values) {
		if (values.empty()) {
			return true;
		}
		std::vector<uint8_t> bytes(values.size() * 4);
		for (size_t i = 0; i < values.size(); ++i) {
			std::memcpy(bytes.data() + i * 4, &values[i], 4);
		}
		return writeMemory(startAddress, bytes.data(), static_cast<uint16_t>(bytes.size()));
	}

	bool writeMemory(uint32_t startAddress, const uint8_t* data, uint16_t bytes) {
		if (!claimed_) {
			std::cerr << "Interface not claimed" << std::endl;
			return false;
		}
		if (bytes == 0) {
			return true;
		}

		const size_t headerSize = sizeof(UVCPWriteMemoryCmd) - sizeof(((UVCPWriteMemoryCmd*)0)->data);
		std::vector<uint8_t> tx(headerSize + bytes, 0);
		auto* cmd = reinterpret_cast<UVCPWriteMemoryCmd*>(tx.data());
		cmd->header.magic = UVCPConstants::MAGIC;
		cmd->header.flags = UVCPConstants::FLAGS_REQUEST_ACK;
		cmd->header.command = UVCPConstants::COMMAND_WRITE_MEMORY_CMD;
		cmd->header.size = sizeof(cmd->address) + bytes;
		cmd->header.id = nextRequestId();
		cmd->address = static_cast<uint64_t>(startAddress);
		std::memcpy(cmd->data, data, bytes);

		if (!bulkSend(tx.data(), static_cast<int>(tx.size()))) {
			return false;
		}

		std::array<uint8_t, TY_UVCP_MAX_MSG_LEN> buffer{};
		int pendingLoops = 0;
		constexpr int kMaxPendingLoops = 5;
		while (true) {
			if (!bulkReceive(buffer.data(), static_cast<int>(buffer.size()))) {
				return false;
			}
			auto* hdr = reinterpret_cast<UVCPHeader*>(buffer.data());
			if (hdr->magic != UVCPConstants::MAGIC) {
				std::cerr << "Invalid ACK magic" << std::endl;
				return false;
			}
			if (hdr->id != cmd->header.id) {
				std::cerr << "ACK id mismatch: got " << hdr->id << ", expected " << cmd->header.id
						  << std::endl;
				return false;
			}
			if (hdr->command == UVCPConstants::COMMAND_PENDING_ACK) {
				auto* p = reinterpret_cast<UVCPPendingAck*>(buffer.data());
				const uint16_t waitMs = p->timeout_ms;
				if (++pendingLoops > kMaxPendingLoops) {
					std::cerr << "Too many PENDING_ACK responses" << std::endl;
					return false;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(waitMs ? waitMs : 1));
				continue;
			}
			if (hdr->command != UVCPConstants::COMMAND_WRITE_MEMORY_ACK) {
				std::cerr << "Unexpected ACK command: 0x" << std::hex << hdr->command << std::dec
						  << std::endl;
				return false;
			}

			auto* ack = reinterpret_cast<UVCPWriteMemoryAck*>(buffer.data());
			if (ack->bytes_written != bytes) {
				std::cerr << "Write bytes mismatch: got " << ack->bytes_written << ", expected "
						  << bytes << std::endl;
				return false;
			}
			return true;
		}
	}

	void shutdown() {
		if (handle_ && claimed_) {
			libusb_release_interface(handle_, interfaceNumber_);
			claimed_ = false;
		}
		if (handle_) {
			libusb_close(handle_);
			handle_ = nullptr;
		}
		if (ctx_) {
			libusb_exit(ctx_);
			ctx_ = nullptr;
		}
	}

  private:
	bool bulkSend(const void* data, int length) {
		int transferred = 0;
		const int rc = libusb_bulk_transfer(handle_, bulkOut_,
											reinterpret_cast<unsigned char*>(const_cast<void*>(data)),
											length, &transferred, kTransferTimeoutMs);
		if (rc != LIBUSB_SUCCESS || transferred != length) {
			std::cerr << "Bulk OUT failed: " << libusb_error_name(rc)
					  << ", bytes=" << transferred << '/' << length << std::endl;
			return false;
		}
		return true;
	}

	bool bulkReceive(void* data, int length) {
		int transferred = 0;
		const int rc = libusb_bulk_transfer(handle_, bulkIn_,
											reinterpret_cast<unsigned char*>(data), length,
											&transferred, kTransferTimeoutMs);
		if (rc != LIBUSB_SUCCESS) {
			std::cerr << "Bulk IN failed: " << libusb_error_name(rc) << std::endl;
			return false;
		}
		if (transferred <= 0) {
			std::cerr << "Bulk IN returned " << transferred << " bytes" << std::endl;
			return false;
		}

		return true;
	}

	uint16_t nextRequestId() { return ++requestId_; }

	libusb_context* ctx_ = nullptr;
	libusb_device_handle* handle_ = nullptr;
	uint8_t interfaceNumber_ = 0;
	uint8_t bulkOut_ = 0;
	uint8_t bulkIn_ = 0;
	bool claimed_ = false;
	uint16_t requestId_ = 0;
	};

	std::vector<std::string> splitTokens(const std::string& line);

	class TerminalClient {
  public:
	explicit TerminalClient(U3VDevice& dev) : device_(dev) {}
	uint32_t getVersion() const { return version_; }

	bool initialize() {
		if (initialized_) {
			return true;
		}
		std::vector<uint32_t> regs;
		if (!device_.readRegisters(kTerminalBaseAddr, 2, regs)) {
			std::cerr << "Failed to read terminal header" << std::endl;
			return false;
		}
		if (regs[0] != kTerminalMagic) {
			std::cerr << "Unexpected terminal magic 0x" << std::hex << regs[0]
					  << ", expected 0x" << kTerminalMagic << std::dec << std::endl;
			return false;
		}
		version_ = regs[1];
		// also read explicit version register if available
		uint32_t verReg = 0;
		if (device_.readRegisters(kTerminalVersionAddr, 1, regs)) {
			verReg = regs[0];
			if (verReg != 0) {
				version_ = verReg;
			}
		}
		chunkHint_ = readRegisterOr(kTerminalChunkHintAddr, chunkHint_);
		if (chunkHint_ == 0) {
			chunkHint_ = 512;
		}
		initialized_ = true;
		return true;
	}

	bool ensureSession() {
		if (!initialized_ && !initialize()) {
			return false;
		}
		if (!ensureAuth()) {
			return false;
		}
		uint32_t status = 0;
		if (!readRegister(kTerminalStatusAddr, status)) {
			return false;
		}
		if (status & kStatusReady) {
			return true;
		}
		uint32_t ctrl = kCtrlStart | kCtrlClearFlags;
		if (echoEnabled_) {
			ctrl |= kCtrlEchoEnable;
		} else {
			ctrl |= kCtrlEchoDisable;
		}
		if (!writeRegister(kTerminalStatusAddr, ctrl)) {
			return false;
		}
		auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
		while (std::chrono::steady_clock::now() < deadline) {
			if (!readRegister(kTerminalStatusAddr, status)) {
				return false;
			}
			if (status & kStatusReady) {
				return true;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
		}
		std::cerr << "Timed out waiting for terminal session" << std::endl;
		return false;
	}

	bool reset() {
		if (!initialize()) {
			return false;
		}
		// do not clear auth
		uint32_t ctrl = kCtrlReset | kCtrlClearFlags;
		if (echoEnabled_) {
			ctrl |= kCtrlEchoEnable;
		} else {
			ctrl |= kCtrlEchoDisable;
		}
		if (!writeRegister(kTerminalStatusAddr, ctrl)) {
			return false;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
		return ensureSession();
	}

	void setPassword(const std::string& pass) { password_ = pass; }

	bool ensureAuth() {
		uint32_t authed = 0;
		if (!readRegister(kTerminalAuthStatusAddr, authed)) {
			return false;
		}
		if (authed) return true;
		std::string pw = password_;
		if (pw.empty()) {
			std::cerr << "Terminal locked: provide password via --password" << std::endl;
			return false;
		}
		// write password to auth buffer (as C-string with NUL terminator recommended but not required)
		if (!device_.writeMemory(kTerminalAuthBufAddr,
								 reinterpret_cast<const uint8_t*>(pw.data()),
								 static_cast<uint16_t>(pw.size()))) {
			return false;
		}
		if (!device_.writeRegister(kTerminalAuthCmdAddr, 1)) {
			return false;
		}
		// re-check
		if (!readRegister(kTerminalAuthStatusAddr, authed)) {
			return false;
		}
		if (!authed) {
			std::cerr << "Authentication failed" << std::endl;
			return false;
		}
		return true;
	}

	bool lock(){
		if (!device_.writeRegister(kTerminalAuthCmdAddr, 0)) {
			return false;
		}
		return true;
	}

	bool sendCommand(const std::string& command) {
		if (!ensureSession()) {
			return false;
		}
		std::string payload = command;
		if (payload.empty() || payload.back() != '\n') {
			payload.push_back('\n');
		}

		size_t offset = 0;
		while (offset < payload.size()) {
			const size_t chunk = std::min<size_t>(chunkHint_, payload.size() - offset);
			if (!device_.writeMemory(kTerminalDataAddr,
									 reinterpret_cast<const uint8_t*>(payload.data() + offset),
									 static_cast<uint16_t>(chunk))) {
				return false;
			}
			offset += chunk;
		}
		return true;
	}

	bool drainOutput(std::string& out,
					 std::chrono::milliseconds idleTimeout = std::chrono::milliseconds(200),
					 std::chrono::milliseconds maxWait = std::chrono::seconds(5)) {
		if (!ensureSession()) {
			return false;
		}

		auto lastData = std::chrono::steady_clock::now();
		auto deadline = std::chrono::steady_clock::now() + maxWait;
		bool warnedOverflow = false;

		while (std::chrono::steady_clock::now() < deadline) {
			uint32_t status = 0;
			if (!readRegister(kTerminalStatusAddr, status)) {
				return false;
			}
			if ((status & kStatusOverflow) && !warnedOverflow) {
				std::cerr << "Warning: terminal output overflowed, some bytes dropped" << std::endl;
				warnedOverflow = true;
			}
			if (status & kStatusError) {
				std::cerr << "Terminal reported error bit" << std::endl;
			}

			uint32_t available = 0;
			if (!readRegister(kTerminalAvailAddr, available)) {
				return false;
			}
			if (available == 0) {
				if (std::chrono::steady_clock::now() - lastData > idleTimeout) {
					break;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(50));
				continue;
			}

			const uint32_t toRead = static_cast<uint32_t>(std::min<uint32_t>(available, chunkHint_));
			std::vector<uint8_t> buffer;
			if (!device_.readMemory(kTerminalDataAddr, static_cast<uint16_t>(toRead), buffer)) {
				return false;
			}
			out.append(buffer.begin(), buffer.end());
			lastData = std::chrono::steady_clock::now();
		}
		return true;
	}

	bool runOnce(const std::string& command) {
		bool handled = false;
		if (!handleFileTransferCommand(command, handled)) {
			return false;
		}
		if (handled) {
			return true;
		}
		if (!sendCommand(command)) {
			return false;
		}
		std::string output;
		if (!drainOutput(output)) {
			return false;
		}
		if (!output.empty()) {
			std::cout << output << std::flush;
		}
		return true;
	}

	bool interactiveLoopV1() {
		if (!ensureSession()) {
			return false;
		}
		std::cout << "Interactive shell ready (firmware version 0x" << std::hex << version_
				  << std::dec << "). Type 'exit' to quit." << std::endl;

		std::string warmup;
		drainOutput(warmup, std::chrono::milliseconds(50), std::chrono::milliseconds(500));
		if (!warmup.empty()) {
			std::cout << warmup << std::flush;
		}
		
		{
			if (!sendCommand("cd /root")) {
				return false;
			}
			std::string output;
			if (!drainOutput(output)) {
				return false;
			}
			if (!output.empty()) {
				std::cout << output << std::flush;
			}
		}

		std::string line;
		while (true) {
			if (!std::getline(std::cin, line)) {
				std::cout << std::endl;
				break;
			}
			if (line == "exit" || line == "quit") {
				break;
			}
			bool handled = false;
			handleFileTransferCommand(line, handled);
			if (handled) {
				if (!sendCommand(" ")) {
					return false;
				}
				std::string output;
				if (!drainOutput(output)) {
					return false;
				}
				if (!output.empty()) {
					std::cout << output << std::flush;
				}
				continue;
			}
			if (!sendCommand(line)) {
				return false;
			}
			std::string output;
			if (!drainOutput(output)) {
				return false;
			}
			if (!output.empty()) {
				std::cout << output << std::flush;
			}
		}
		return true;
	}

	bool interactiveLoopV2() {
		if (!ensureSession()) {
			return false;
		}
		std::cout << "Interactive shell ready (firmware version 0x" << std::hex << version_
				  << std::dec << "). Type 'exit' to quit." << std::endl;

		// Drain any warmup output.
		std::string warmup;
		drainOutput(warmup, std::chrono::milliseconds(50), std::chrono::milliseconds(500));
		if (!warmup.empty()) {
			std::cout << warmup << std::flush;
		}

		// Send initial working directory.
		if (!sendCommand("cd /root")) {
			return false;
		}
		std::string initOutput;
		if (!drainOutput(initOutput)) {
			return false;
		}
		if (!initOutput.empty()) {
			std::cout << initOutput << std::flush;
		}

		// Configure local terminal to raw mode for byte-stream behaviour.
		StdinState stdinState{};
		if (!setStdinRaw(stdinState)) {
			return false;
		}
		auto restore = [&stdinState]() { restoreStdin(stdinState); };
		// Ensure restoration even if we early-return below.
		struct Guard {
			std::function<void()> fn;
			~Guard() { if (fn) fn(); }
		} guard{restore};

		constexpr char kExitKey = 0x1d; // Ctrl+]
		std::array<char, 256> inBuf{};
		auto lastPoll = std::chrono::steady_clock::now();
		std::string currentLine;

		while (true) {
			// 1) Read from stdin (non-blocking-ish by using small timeout or polling).
			ssize_t n = 0;
#ifndef _WIN32
			fd_set rfds;
			FD_ZERO(&rfds);
			FD_SET(STDIN_FILENO, &rfds);
			timeval tv{};
			// Short timeout to also poll device output regularly.
			tv.tv_sec = 0;
			tv.tv_usec = 20000; // 20 ms
			int sel = select(STDIN_FILENO + 1, &rfds, nullptr, nullptr, &tv);
			if (sel > 0 && FD_ISSET(STDIN_FILENO, &rfds)) {
				n = ::read(STDIN_FILENO, inBuf.data(), inBuf.size());
			}
#else
			if (_kbhit()) {
				int idx = 0;
				while (_kbhit() && idx < static_cast<int>(inBuf.size())) {
					int ch = _getch();
					inBuf[static_cast<size_t>(idx++)] = static_cast<char>(ch);
				}
				n = idx;
			} else {
				std::this_thread::sleep_for(std::chrono::milliseconds(20));
			}
#endif
			if (n > 0) {
				// Check for Ctrl+] to exit locally.
				bool exitRequested = false;
				std::vector<uint8_t> toSend;
				toSend.reserve(static_cast<size_t>(n));
				for (ssize_t i = 0; i < n; ++i) {
					unsigned char ch = static_cast<unsigned char>(inBuf[static_cast<size_t>(i)]);
					if (ch == static_cast<unsigned char>(kExitKey)) {
						exitRequested = true;
						continue;
					}
					// Maintain a simple local line buffer to intercept "exit".
					if (ch == '\r' || ch == '\n') {
						// End of line: check if the line is exactly "exit".
						if (currentLine == "exit") {
							// Do not send this line to remote, just exit locally.
							toSend.push_back('\b');
							toSend.push_back('\b');
							toSend.push_back('\b');
							toSend.push_back('\b');
							device_.writeMemory(kTerminalDataAddr, toSend.data(),
											static_cast<uint16_t>(toSend.size()));
							toSend.clear();
							exitRequested = true;
							currentLine.clear();
							continue;
						} else {
							if(tryHandleFileTransferCommand(currentLine)){
								bool handle = false;
								for(int i =0;i<currentLine.size();i++){
									toSend.push_back('\b');
								}
								toSend.push_back('\n');
								device_.writeMemory(kTerminalDataAddr, toSend.data(),
											static_cast<uint16_t>(toSend.size()));
								toSend.clear();
								std::cout << std::endl;
								handleFileTransferCommand(currentLine,handle);
								currentLine.clear();
								continue;	
							}
						}
						currentLine.clear();
					} else if (ch == 0x7f || ch == '\b') {
						// Handle backspace locally for the line buffer.
						if (!currentLine.empty()) {
							currentLine.pop_back();
						}
					} else if (std::isprint(ch)) {
						currentLine.push_back(static_cast<char>(ch));
					}
					toSend.push_back(ch);
				}
				if (!toSend.empty()) {
					if (!device_.writeMemory(kTerminalDataAddr, toSend.data(),
								   static_cast<uint16_t>(toSend.size()))) {
						return false;
					}
				}
				if (exitRequested) {
					break;
				}
			}

			// 2) Periodically poll device output.
			auto now = std::chrono::steady_clock::now();
			if (now - lastPoll >= std::chrono::milliseconds(10)) {
				std::string out;
				if (!drainOutput(out, std::chrono::milliseconds(10), std::chrono::milliseconds(10))) {
					return false;
				}
				if (!out.empty()) {
#ifndef _WIN32
					int wr = ::write(STDOUT_FILENO, out.data(), out.size());
					(void)wr;
#else
					std::cout.write(out.data(), out.size());
					std::cout.flush();
#endif
				}
				lastPoll = now;
			}
		}

		return true;
	}

	bool tryHandleFileTransferCommand(const std::string& line) {
		auto tokens = splitTokens(line);
		if (tokens.empty()) {
			return false;
		}
		const std::string& op = tokens[0];
		if (op != "u3vget" && op != "u3vput") {
			return false;
		}
		return true;
	}

	bool handleFileTransferCommand(const std::string& line, bool& handled) {
		handled = false;
		auto tokens = splitTokens(line);
		if (tokens.empty()) {
			return true;
		}
		const std::string& op = tokens[0];
		if (op != "u3vget" && op != "u3vput") {
			return true;
		}
		handled = true;
		if (op == "u3vget") {
			if (tokens.size() != 3) {
				std::cerr << "Usage: u3vget <remote-path> <local-path>" << std::endl;
				return true;
			}
			return performFileDownload(tokens[1], tokens[2]);
		}
		if (tokens.size() != 3) {
			std::cerr << "Usage: u3vput <local-path> <remote-path>" << std::endl;
			return true;
		}
		return performFileUpload(tokens[1], tokens[2]);
	}
	void setEchoEnabled(bool enable) { echoEnabled_ = enable; }

  private:
	struct StdinState {
#ifdef _WIN32
		DWORD origMode = 0;
		bool hasMode = false;
#else
		termios orig{};
#endif
	};

	static bool setStdinRaw(StdinState& state) {
#ifdef _WIN32
		HANDLE hIn = GetStdHandle(STD_INPUT_HANDLE);
		if (hIn == INVALID_HANDLE_VALUE || hIn == nullptr) {
			return false;
		}
		DWORD mode = 0;
		if (!GetConsoleMode(hIn, &mode)) {
			return false;
		}
		state.origMode = mode;
		state.hasMode = true;
		mode &= ~(ENABLE_ECHO_INPUT | ENABLE_LINE_INPUT | ENABLE_PROCESSED_INPUT);
		mode |= ENABLE_EXTENDED_FLAGS;
		if (!SetConsoleMode(hIn, mode)) {
			return false;
		}
		return true;
#else
		if (tcgetattr(STDIN_FILENO, &state.orig) != 0) {
			std::perror("tcgetattr");
			return false;
		}
		termios raw = state.orig;
		raw.c_lflag &= ~(ICANON | ECHO | ISIG);
		raw.c_iflag &= ~(IXON | ICRNL);
		raw.c_cc[VMIN] = 1;
		raw.c_cc[VTIME] = 0;
		if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
			std::perror("tcsetattr");
			return false;
		}
		return true;
#endif
	}

	static void restoreStdin(const StdinState& state) {
#ifdef _WIN32
		if (!state.hasMode) {
			return;
		}
		HANDLE hIn = GetStdHandle(STD_INPUT_HANDLE);
		if (hIn == INVALID_HANDLE_VALUE || hIn == nullptr) {
			return;
		}
		SetConsoleMode(hIn, state.origMode);
#else
		if (tcsetattr(STDIN_FILENO, TCSANOW, &state.orig) != 0) {
			std::perror("tcsetattr");
		}
#endif
	}

	uint32_t readRegisterOr(uint32_t addr, uint32_t fallback) {
		uint32_t value = 0;
		if (readRegister(addr, value)) {
			return value;
		}
		return fallback;
	}

	bool readRegister(uint32_t addr, uint32_t& value) {
		std::vector<uint32_t> vals;
		if (!device_.readRegisters(addr, 1, vals)) {
			return false;
		}
		value = vals[0];
		return true;
	}

	bool writeRegister(uint32_t addr, uint32_t value) {
		return device_.writeRegister(addr, value);
	}

	bool prepareFilePath(const std::string& remotePath) {
		if (remotePath.empty()) {
			std::cerr << "Remote path must not be empty" << std::endl;
			return false;
		}
		if (!sendFileCommand(kFileCmdReset)) {
			return false;
		}
		if (remotePath.size() >= kTerminalFilePathCapacity) {
			std::cerr << "Remote path exceeds " << kTerminalFilePathCapacity - 1
				  << " bytes limit" << std::endl;
			return false;
		}
		std::array<uint8_t, kTerminalFilePathCapacity> buffer{};
		std::memcpy(buffer.data(), remotePath.data(), remotePath.size());
		buffer[remotePath.size()] = 0;
		return device_.writeMemory(kTerminalFilePathAddr, buffer.data(),
						 static_cast<uint16_t>(buffer.size()));
	}

	bool sendFileCommand(uint32_t cmd) {
		return writeRegister(kTerminalFileCmdAddr, cmd);
	}

	bool readFileStatus(uint32_t& status) {
		return readRegister(kTerminalFileStatusAddr, status);
	}

	bool readFileResult(int& result) {
		uint32_t value = 0;
		if (!readRegister(kTerminalFileResultAddr, value)) {
			return false;
		}
		result = static_cast<int>(value);
		return true;
	}

	bool waitForFileOpen(uint32_t modeBit, std::chrono::milliseconds timeout) {
		auto deadline = std::chrono::steady_clock::now() + timeout;
		while (std::chrono::steady_clock::now() < deadline) {
			uint32_t status = 0;
			if (!readFileStatus(status)) {
				return false;
			}
			if (status & modeBit) {
				return true;
			}
			if (status & kFileStatusError) {
				return checkFileError("open file");
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
		std::cerr << "Timed out waiting for file channel" << std::endl;
		return false;
	}

	bool performFileDownload(const std::string& remotePath, const std::string& localPath) {
		if (!ensureSession()) {
			return false;
		}
		if (!prepareFilePath(remotePath)) {
			return false;
		}
		if (!sendFileCommand(kFileCmdOpenRead)) {
			return false;
		}
		if (!waitForFileOpen(kFileStatusReading, std::chrono::milliseconds(500))) {
			closeFileChannel();
			return false;
		}
		uint64_t remoteSize = 0;
		readFileSize(remoteSize);
		std::ofstream ofs(localPath, std::ios::binary | std::ios::trunc);
		if (!ofs) {
			std::cerr << "Unable to open local file '" << localPath << "' for writing"
				  << std::endl;
			closeFileChannel();
			return false;
		}
		uint64_t bytesReceived = 0;
		bool progressPrinted = false;
		bool success = true;
		while (success) {
			uint32_t avail = 0;
			if (!readFileDataAvail(avail)) {
				success = false;
				break;
			}
			if (avail == 0) {
				uint32_t status = 0;
				if (!readFileStatus(status)) {
					success = false;
					break;
				}
				if (status & kFileStatusError) {
					success = checkFileError("u3vget");
					break;
				}
				if (status & kFileStatusEof) {
					break;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
				continue;
			}
			std::vector<uint8_t> buffer;
			if (!device_.readMemory(kTerminalFileDataAddr, static_cast<uint16_t>(avail), buffer)) {
				success = false;
				break;
			}
			ofs.write(reinterpret_cast<const char*>(buffer.data()),
					static_cast<std::streamsize>(avail));
			if (!ofs) {
				std::cerr << "Failed writing to local file '" << localPath << "'" << std::endl;
				success = false;
				break;
			}
			bytesReceived += avail;
			if (remoteSize > 0) {
				progressPrinted = true;
				double pct = remoteSize ? (100.0 * static_cast<double>(bytesReceived) / static_cast<double>(remoteSize)) : 0.0;
				std::cout << '\r' << "Downloading: " << bytesReceived << '/' << remoteSize << " (" << std::fixed << std::setprecision(1) << pct << "%)" << std::flush;
			} else {
				progressPrinted = true;
				std::cout << '\r' << "Downloading: " << bytesReceived << " bytes" << std::flush;
			}
		}
		if (!closeFileChannel()) {
			success = false;
		}
		if (progressPrinted) {
			std::cout << '\n';
		}
		if (success) {
			std::cout << "Downloaded '" << remotePath << "' -> '" << localPath << "'";
			if (remoteSize != 0) {
				std::cout << " (" << remoteSize << " bytes)";
			}
			std::cout << std::endl;
		}
		return success;
	}

	bool performFileUpload(const std::string& localPath, const std::string& remotePath) {
		std::ifstream ifs(localPath, std::ios::binary);
		if (!ifs) {
			std::cerr << "Unable to open local file '" << localPath << "'" << std::endl;
			return false;
		}
		ifs.seekg(0, std::ios::end);
		std::streampos end = ifs.tellg();
		ifs.clear();
		ifs.seekg(0, std::ios::beg);
		uint64_t totalBytes = end >= 0 ? static_cast<uint64_t>(end) : 0;
		if (!ensureSession()) {
			return false;
		}
		if (!prepareFilePath(remotePath)) {
			return false;
		}
		if (!sendFileCommand(kFileCmdOpenWrite)) {
			return false;
		}
		if (!waitForFileOpen(kFileStatusWriting, std::chrono::milliseconds(500))) {
			closeFileChannel();
			return false;
		}
		std::array<char, kTerminalFileDataWindow> buffer{};
		uint64_t bytesSent = 0;
		bool progressPrinted = false;
		bool success = true;
		while (ifs && success) {
			ifs.read(buffer.data(), buffer.size());
			std::streamsize got = ifs.gcount();
			if (got <= 0) {
				break;
			}
			if (!device_.writeMemory(kTerminalFileDataAddr,
							reinterpret_cast<const uint8_t*>(buffer.data()),
							static_cast<uint16_t>(got))) {
				success = false;
				break;
			}
			uint32_t status = 0;
			if (!readFileStatus(status)) {
				success = false;
				break;
			}
			if (status & kFileStatusError) {
				success = checkFileError("u3vput");
				break;
			}
			bytesSent += static_cast<uint64_t>(got);
			if (totalBytes > 0) {
				progressPrinted = true;
				double pct = totalBytes ? (100.0 * static_cast<double>(bytesSent) / static_cast<double>(totalBytes)) : 0.0;
				std::cout << '\r' << "Uploading:   " << bytesSent << '/' << totalBytes << " (" << std::fixed << std::setprecision(1) << pct << "%)" << std::flush;
			} else {
				progressPrinted = true;
				std::cout << '\r' << "Uploading:   " << bytesSent << " bytes" << std::flush;
			}
		}
		if (!closeFileChannel()) {
			success = false;
		}
		if (progressPrinted) {
			std::cout << '\n';
		}
		if (success) {
			std::cout << "Uploaded '" << localPath << "' -> '" << remotePath << "'";
			if (totalBytes != 0) {
				std::cout << " (" << totalBytes << " bytes)";
			}
			std::cout << std::endl;
		}
		return success;
	}

	bool closeFileChannel() {
		if (!sendFileCommand(kFileCmdClose)) {
			return false;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		return checkFileError("file transfer");
	}

	bool checkFileError(const std::string& context) {
		uint32_t status = 0;
		if (!readFileStatus(status)) {
			return false;
		}
		if ((status & kFileStatusError) == 0) {
			return true;
		}
		int err = 0;
		if (!readFileResult(err)) {
			return false;
		}
		std::cerr << context << " failed";
		if (err != 0) {
			std::cerr << ": errno=" << err << " (" << std::strerror(err) << ")";
		}
		std::cerr << std::endl;
		return false;
	}

	bool readFileDataAvail(uint32_t& avail) {
		return readRegister(kTerminalFileDataAvailAddr, avail);
	}

	bool readFileSize(uint64_t& size) {
		uint32_t low = 0;
		uint32_t high = 0;
		if (!readRegister(kTerminalFileSizeLowAddr, low)) {
			return false;
		}
		if (!readRegister(kTerminalFileSizeHighAddr, high)) {
			return false;
		}
		size = (static_cast<uint64_t>(high) << 32) | low;
		return true;
	}

	U3VDevice& device_;
	bool initialized_ = false;
	uint32_t version_ = 0;
	uint32_t chunkHint_ = 4096;
	std::string password_;
	bool echoEnabled_ = true;
};

std::string joinArguments(int argc, char** argv, int startIndex) {
	std::ostringstream oss;
	for (int i = startIndex; i < argc; ++i) {
		if (i > startIndex) {
			oss << ' ';
		}
		oss << argv[i];
	}
	return oss.str();
}

std::vector<std::string> splitTokens(const std::string& line) {
	std::istringstream iss(line);
	std::vector<std::string> tokens;
	std::string token;
	while (iss >> token) {
		tokens.push_back(token);
	}
	return tokens;
}

std::vector<std::string> splitTokens(const std::string& line);

void printUsage(const char* exe) {
	std::cout << "Usage: " << exe << " [options] [command]\n"
			  << "Options:\n"
			  << "  -c,  --command <cmd>             Execute a single command then exit\n"
			  << "  -i,  --interactive               Force interactive mode (default if no command)\n"
			  << "  -get <remote-path> <local-path>  Execute get file command then exit\n"
			  << "  -put <local-path> <remote-path>  Execute put file command then exit\n"
			  << "  -r,  --reset                     Reset terminal session before use\n"
			  << "  -p,  --password <pwd>            Password for unlocking terminal (or use TY_TERM_PASS)\n"
		  	  << "  -id, --id <serial>               Match device by USB serial number (iSerial)\n"
		      << "                                   (omit to be prompted when multiple devices exist)\n"
			  << "       --vid <id>                  USB vendor ID (e.g., 0x04b4)\n"
			  << "       --pid <id>                  USB product ID (e.g., 0x1003)\n"
			  << "  -h,  --help                      Show this message\n";
}

}  // namespace

int main(int argc, char** argv) {
	bool interactive = true;
	int interactiveMode = 2;
	bool resetSession = false;
	std::string singleCommand;
	std::string password;
	std::string serialFilter;
	uint16_t vendorId = kVendorId;
	uint16_t productId = kProductId;

	auto parseU16 = [](const std::string& s, uint16_t& out) -> bool {
		try {
			unsigned long v = std::stoul(s, nullptr, 0); // auto-detect base (0x..., 0...)
			if (v > 0xFFFF) return false;
			out = static_cast<uint16_t>(v);
			return true;
		} catch (...) {
			return false;
		}
	};

	for (int i = 1; i < argc; ++i) {
		std::string arg = argv[i];
		if (arg == "-h" || arg == "--help") {
			printUsage(argv[0]);
			return EXIT_SUCCESS;
		} else if (arg == "-i" || arg == "--interactive") {
			if (i + 1 >= argc) {
				std::cerr << "--command requires an argument" << std::endl;
				return EXIT_FAILURE;
			}
			interactive = true;
			uint16_t v = 0;
			if (!parseU16(argv[++i], v)) {
				std::cerr << "Invalid interactive mode value" << std::endl;
				return EXIT_FAILURE;
			}
			interactiveMode = v;
			singleCommand.clear();
		} else if (arg == "-r" || arg == "--reset") {
			resetSession = true;
		} else if (arg == "-c" || arg == "--command") {
			if (i + 1 >= argc) {
				std::cerr << "--command requires an argument" << std::endl;
				return EXIT_FAILURE;
			}
			singleCommand = argv[++i];
			interactive = false;
		} else if (arg == "-get") {
			if (i + 2 >= argc) {
				std::cerr << "-get requires 2 arguments" << std::endl;
				return EXIT_FAILURE;
			}
			singleCommand = std::string("u3vget ") + argv[i+1] + " " + argv[i+2];
			i+=2;
			interactive = false;
		} else if (arg == "-put") {
			if (i + 2 >= argc) {
				std::cerr << "-put requires 2 arguments" << std::endl;
				return EXIT_FAILURE;
			}
			singleCommand = std::string("u3vput ") + argv[i+1] + " " + argv[i+2];
			i+=2;
			interactive = false;
		} else if (arg == "-p" || arg == "--password") {
			if (i + 1 >= argc) {
				std::cerr << "--password requires an argument" << std::endl;
				return EXIT_FAILURE;
			}
			password = argv[++i];
		} else if (arg == "-id" || arg == "--id") {
			if (i + 1 >= argc) {
				std::cerr << "--id requires an argument" << std::endl;
				return EXIT_FAILURE;
			}
			serialFilter = argv[++i];
		} else if (arg == "--vid") {
			if (i + 1 >= argc) {
				std::cerr << "--vid requires an argument" << std::endl;
				return EXIT_FAILURE;
			}
			uint16_t v = 0;
			if (!parseU16(argv[++i], v)) {
				std::cerr << "Invalid vendor ID value" << std::endl;
				return EXIT_FAILURE;
			}
			vendorId = v;
		} else if (arg == "--pid") {
			if (i + 1 >= argc) {
				std::cerr << "--pid requires an argument" << std::endl;
				return EXIT_FAILURE;
			}
			uint16_t v = 0;
			if (!parseU16(argv[++i], v)) {
				std::cerr << "Invalid product ID value" << std::endl;
				return EXIT_FAILURE;
			}
			productId = v;
		} else {
			singleCommand = joinArguments(argc, argv, i);
			interactive = false;
			break;
		}
	}

	U3VDevice device;
	if (!device.open(vendorId, productId, serialFilter)) {
		return EXIT_FAILURE;
	}

	uint8_t controlInterface = 0, epOut = 0, epIn = 0;
	if (!device.findU3VControlInterface(controlInterface, epOut, epIn)) {
		return EXIT_FAILURE;
	}
	if (!device.claimInterface(controlInterface, epOut, epIn)) {
		return EXIT_FAILURE;
	}

	TerminalClient terminal(device);
	if (!terminal.initialize()) {
		return EXIT_FAILURE;
	}
	if (!password.empty()) {
		terminal.setPassword(password);
	}

	// kTerminal version must be >= 0x00010002 to allow V2 mode
	const uint32_t kMinV2Version = 0x00010002u;
	uint32_t kVersion = terminal.getVersion();
	if (interactiveMode >= 2 && kVersion < kMinV2Version) {
		std::cerr << "kTerminal version 0x" << std::hex << kVersion
			  << " is below 0x" << kMinV2Version
			  << ", falling back to V1 mode" << std::dec << std::endl;
		interactiveMode = 1;
	}
	if (interactive) {
		terminal.setEchoEnabled(interactiveMode == 2);
	} else {
		terminal.setEchoEnabled(false);
	}

	if (resetSession && !terminal.reset()) {
		return EXIT_FAILURE;
	}

	bool ok = false;
	if (interactive) {
		if(interactiveMode == 1){
			ok = terminal.interactiveLoopV1();
		} else if(interactiveMode == 2){
			ok = terminal.interactiveLoopV2();
		} else {
			ok = terminal.interactiveLoopV2();
		}
	} else {
		ok = terminal.runOnce(singleCommand);
	}

	if (!terminal.lock()) {
		return EXIT_FAILURE;
	}

	device.shutdown();
	return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
