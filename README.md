# TX2

 
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

================================================================================
			Install CUDA & cuDNN
================================================================================
   
   Use Jetpack to install CUDA and cuDNN
   Reference: http://huchaowei.com/2017/04/19/TX1%E9%85%8D%E7%BD%AE%E6%95%99%E7%A8%8B/
   

================================================================================
			Install OpenCV3.2
================================================================================
    Source: https://github.com/jetsonhacks/buildOpenCVTX2
    
    $ sudo apt-add-repository universe
    $ sudo apt-get update
    $ sudo apt-get install \
	    libglew-dev \
	    libtiff5-dev \
	    zlib1g-dev \
	    libjpeg-dev \
	    libpng12-dev \
	    libjasper-dev \
	    libavcodec-dev \
	    libavformat-dev \
	    libavutil-dev \
	    libpostproc-dev \
	    libswscale-dev \
	    libeigen3-dev \
	    libtbb-dev \
	    libgtk2.0-dev \
	    pkg-config
    $  sudo apt-get install -y \
    cmake \
    libavcodec-dev \
    libavformat-dev \
    libavutil-dev \
    libeigen3-dev \
    libglew-dev \
    libgtk2.0-dev \
    libgtk-3-dev \
    libjasper-dev \
    libjpeg-dev \
    libpng12-dev \
    libpostproc-dev \
    libswscale-dev \
    libtbb-dev \
    libtiff5-dev \
    libv4l-dev \
    libxvidcore-dev \
    libx264-dev \
    qt5-default \
    zlib1g-dev \
    pkg-config
    $ sudo apt-get install python-dev python-numpy python-py python-pytest
    $ mkdir build
    $ cd build
    $ cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr \
      -D WITH_CUDA=ON \
      -D CUDA_ARCH_BIN=6.2 \
      -D CUDA_ARCH_PTX="" \
      -D ENABLE_FAST_MATH=ON \
      -D CUDA_FAST_MATH=ON \
      -D WITH_CUBLAS=ON \
      -D WITH_LIBV4L=ON \
      -D WITH_GSTREAMER=ON \
      -D WITH_GSTREAMER_0_10=OFF \
      -D WITH_QT=ON \
      -D WITH_OPENGL=OFF \
      ../
     $ make
     $ sudo make install
   

possible issues: GraphCut deprecated in CUDA 7.5 and removed in 8.0
Solution: https://github.com/opencv/opencv/pull/6510/commits/10896129b39655e19e4e7c529153cb5c2191a1db


================================================================================
			Install ROS
================================================================================
    
    install tutorial http://wiki.ros.org/kinetic/Installation/Ubuntu
    possible issues:  Initialize rosdep
	sudo rosdep init
	rosdep update
	But it gives following error.
	ERROR: cannot download default sources list from: https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/sources.list.d/20-default.list
	
	Solution: sudo apt-get install ca-cacert
================================================================================
			Install Caffe OpenPose
================================================================================
     sudo apt-get install libgflags-dev libgoogle-glog-dev liblmdb-dev libatlas-base-dev libleveldb-dev libsnappy-dev
     ~/catkin_ws/build/glog_catkin/glog_src-prefix/src/glog_src$ ln -s /usr/share/automake-1.15/config.guess .
     ~/catkin_ws/build/glog_catkin/glog_src-prefix/src/glog_src$ ln -s /usr/share/automake-1.15/config.sub .

     cd openpose
     make
  
     add ovx.hpp https://github.com/opencv/opencv/blob/master/modules/core/include/opencv2/core/ovx.hpp
    
================================================================================
			Install TensorFlow
================================================================================
      https://github.com/lhelontra/tensorflow-on-arm/releases
      sudo pip3 install --upgrade tensorflow-1.3.1-cp35-none-linux_aarch64.whl
