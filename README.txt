=============================================================================
	     ADVANTECH CAN communication cards socketcan driver
		   Installation & Program Guide for Linux 2.6.33 and newer 
			   (Tested on various kernels from 3.2 to 4.2)
	            Copyright (C) 2020, ADVANTECH Co, Ltd.
=============================================================================
Contents

1. Introduction
2. Test environment
3. System Requirement
4. Installation
5. User space program for this driver

-----------------------------------------------------------------------------
1. Introduction
   
   ADVANTECH CAN communication cards socketcan driver,

   Now it supports following boards.
    
    - PCI-1680          : 2 port Isolated PCI CAN-bus Card.
    - MIC-3680	        : 2 port Isolated PCI CAN-bus Card.
    - PCM-3680I	        : 2 port Isolated PCI CAN-bus Card.
    - ADVANTECH C001 CAN Card (1 PORT)      : 1 port Isolated PCI CAN bus Card.
    - ADVANTECH C002 CAN Card (2 PORT)     : 2 port Isolated PCI CAN bus Card.
    - ADVANTECH C004 CAN Card (4 PORT)     : 4 port Isolated PCI CAN bus Card.
    - ADVANTECH C101 CAN Card (1 PORT, support CANopen)     : 1 port Isolated PCI CAN bus Card and support CANopen.
    - ADVANTECH C102 CAN Card (2 PORT, support CANopen)     : 2 port Isolated PCI CAN bus Card and support CANopen.
    - ADVANTECH C104 CAN Card (4 PORT, support CANopen)     : 4 port Isolated PCI CAN bus Card and support CANopen.
    - ADVANTECH C201 CAN Card (1 PORT)     : 1 port Isolated PCI CAN bus Card.
    - ADVANTECH C202 CAN Card (2 PORT)     : 2 port Isolated PCI CAN bus Card.
    - ADVANTECH C204 CAN Card (4 PORT)     : 4 port Isolated PCI CAN bus Card.
    - ADVANTECH C301 CAN Card (1 PORT, support CANopen)     : 1 port Isolated PCI CAN bus Card and support CANopen.
    - ADVANTECH C302 CAN Card (2 PORT, support CANopen)     : 2 port Isolated PCI CAN bus Card and support CANopen.
    - ADVANTECH C304 CAN Card (4 PORT, support CANopen)     : 4 port Isolated PCI CAN bus Card and support CANopen.

	 
   This driver supports Linux Kernel 2.6.33 and newer Intel x86 hardware 
   platform. Any problem occurs, please contact ADVANTECH at 
   support@ADVANTECH.com.tw.

   This driver and utilities are published in form of source code under
   GNU General Public License in this version. Please refer to GNU General
   Public License announcement in each source code file for more detail.



2. Test environment
   
 ------------------------------------------ -------------- --------------------------------
|             Distribution                                |     Kernel version            |
 ------------------------------------------ -----------------------------------------------
|  Ubuntu 12.04/14.04/15.10/18.04(Desktop)/20.04(Desktop) |      3.2/3.13/4.2/4.15.0/5.11 |
 ------------------------------------------ -----------------------------------------------
|  Redhat Enterprise Linux Server 7.2                     |         3.10.0                |
 ------------------------------------------ -----------------------------------------------
|  OpenSUSE 15.0                                          |         4.12.14               |
 ------------------------------------------ -----------------------------------------------
|  Debian 9.5                                             |         4.9.0                 |
 ------------------------------------------ -----------------------------------------------
|  CentOS 7.6                                             |         3.10.0                |
 ------------------------------------------ -----------------------------------------------
   
   warning : fedora20 and redhat7.2 default not enable SocketCAN
   

3. System Requirement
   - Hardware platform: Intel x86 
   - Kernel version:  2.6.33 and newer


4. Installation

   4.1 Hardware installation
       Just install the card into your computer, please refer to hardware
       installation procedure in the User's Manual.
     
   4.2 Driver files 
       The driver file may be obtained from CD-ROM or www.advantech.com. The
       first step, anyway, is to copy driver file "advSocketcan_vx.xx.tar.gz" into 
       specified directory. e.g. /can. Then execute commands to decompress as below.

       # tar -zxvf advSocketcan_vx.x.x.x.tar.gz
      
   4.3 Compile and install CAN Linux driver
       Module driver is easiest way to compile. In this part, both build 
       method for kernel 3.13 and newer kernel are introduced.

       4.3.1 Build driver with the current running kernel source
          Find "Makefile" in /workdir/advsocketcan/driver/, then run
          # make

	   The driver files "advsocketcan.ko advcan_sja1000.ko"  will be properly compiled.

          Run following command to install the builded driver to the property
          system directory.
          # make install

	   4.3.2  Load driver automatically
		  Open /etc/modules file. Enter the following command: “gedit /etc/modules”.
	      and add the following content
	   
	       lp
		   can
		   can_dev
		   can_raw
	       advsocketcan
		   advcan_sja1000

	      The name which is needed to automatically load was added to this file. then close and save the file.


   4.4 Create device driver file
	   Ensure you have inserted the advsocketcan and advcansja1000 module in the kernel (which can
	   be show by the lsmod command)
	   
	   Use "ip link list" command to see whether the CAN device name is create in net port

	   If device list, you need to create the CAN device and set baudrate manually as follows:
   	   Use the command:
   	
	   "ip link set can0 up type can bitrate 125000"

   4.5 Driver remove.
       You can use the following command to remove the driver from the system,
	
	   # rmmod advsocketcan 
       # rmmod advcan_sja1000     
	
       or 
	   
	   # cd /workdir/advsocketcan/drvier
       # make uninstall
	
               
5. User space program for this driver
   
   5.1 Examples Reference
   
		5.1.1 
		# cd /workdir/advsocketcan/example
		# make
       These are some simple examples to test the communication between two CAN channels.

        5.1.1 readCAN
	      polling or nonblocking mode to receive CAN message
        5.1.2 sendCAN
	      transmit CAN message



