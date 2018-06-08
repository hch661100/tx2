# tx2
tx2
 
This is a introduction about TX2 setup.

The TX2 we use can be obtained in http://connecttech.com/product/orbitty-carrier-for-nvidia-jetson-tx2-tx1/
The BSP is downloaded in http://www.connecttech.com/ftp/Drivers/CTI-L4T-V112.tgz.

================================================================================
			Installation
================================================================================

	1. Before Installing the BSP you will need to install Jetpack 3.1 on the x86
       host system.  

	2. Copy the CTI-L4T-V###.tgz package into <install_dir>/64_TX2/Linux_for_Tegra/:

    3. Extract the BSP:

		tar -xzf CTI-L4T-V###.tgz

	  (replacing ### with your file name)

	3. Change into the CTI-L4T directory:
		
		cd ./CTI-L4T

	4. Run the install script (as root or sudo) to automatically install
	   the BSP files to the correct locations:

	    sudo ./install.sh

    5. The CTI-L4T BSP is now installed on the host system and it should now be
       able to flash the TX2.

    6. To flash on the tx2 use:
        ./flash.sh <boardname> mmcblk0p1

       Examples:
        ./flash.sh astro-usb3 mmcblk0p1
        ./flash.sh astro-mpcie mmcblk0p1
        ./flash.sh orbitty mmcblk0p1

================================================================================
			Flashing TX2
================================================================================
   
    1. Connect the TX2 and Carrier (or Dev-Kit) to the computer via USB
       Following the instructions in the appropriate manual.
    2. Put the system to be flashed into recovery mode, following the 
       instructions in the appropriate manual
    4. Run ./flash.sh <board.conf> mmcblk0p1 from Linux_for_Tegra directory
    5. Once the flashing has completed, the TX2 will reboot 
    6. To switch between different boards, you will need ot repeat these steps
       There is currently no run time support for switching profiles on the TX2

    Consult KDB344 at for TX2 hardware compatiblity with your carrier.
    http://connecttech.com/resource-center-category/all-kdb-entries/


