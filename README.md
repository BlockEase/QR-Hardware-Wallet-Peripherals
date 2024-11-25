# QR Hardware Wallet Peripherals



**Run Guide**

This program consists of two parts:

   - Hardware Configuration: [QR-Hardware-Wallet-Peripherals](https://github.com/BlockEase/QR-Hardware-Wallet-Peripherals) (this repository)
   - Main Program: [QR-Hardware-Wallet](https://github.com/BlockEase/QR-Hardware-Wallet)

**Steps to Run**

**1. Hardware Peripheral Configuration (Run Once)**

   - Fork [QR-Hardware-Wallet-Peripherals](https://github.com/BlockEase/QR-Hardware-Wallet-Peripherals).
   - Modify [peripherals.h](https://github.com/BlockEase/QR-Hardware-Wallet-Peripherals/blob/main/main/peripherals.h) based on your dev board configuration.
   - Compile and flash the **QR-Hardware-Wallet-Peripherals** firmware to your dev board.
   - Verify that the touchscreen, LCD, and camera module. Follow the on-screen instructions to save the hardware peripheral configuration to the OEM partition.

**2. Run the Main Program**

   - Compile [QR-Hardware-Wallet](https://github.com/BlockEase/QR-Hardware-Wallet) locally or download the pre-built firmware from [Releases](https://github.com/BlockEase/QR-Hardware-Wallet/releases) (not available yet).
   - Flash the firmware to your development board.
