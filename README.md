# PointCloud Delay Check Package
This package was created to test the delay of pointcloud topic communication.

run with: 
`ros2 launch pointcloud_delay_check pointcloud_delay_check.launch.py use_multithread:=true use_intra_process:=False`

## Environment Setup:
1. Install [ROS2 Galactic](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)
2. Change network parameter by creating `/etc/sysctl.d/60_cyclonedds.conf` with the following contents:
```
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
net.core.rmem_max=2147483647
net.core.rmem_default=8388608
net.core.wmem_max=2147483647
net.core.wmem_default=8388608
```
3. Create the configuration file for cyclone DDS (e.g. `${HOME}/cyclonedds_configs/config.xml`). Replace `NetworkInterfaceAddress` with your interface.
```
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>enp5s0</NetworkInterfaceAddress>
            <AllowMulticast>spdp</AllowMulticast>
            <MaxMessageSize>65500B</MaxMessageSize>
            <MaxRexmitMessageSize>65500B</MaxRexmitMessageSize>
            <FragmentSize>4000B</FragmentSize>
        </General>
        <Internal>
            <MinimumSocketReceiveBufferSize>10MB</MinimumSocketReceiveBufferSize>
            <Watermarks>
                <WhcHigh>500kB</WhcHigh>
            </Watermarks>
        </Internal>
        <Tracing>
            <Verbosity>config</Verbosity>
            <OutputFile>/tmp/cyclonedds.log</OutputFile>
        </Tracing>
    </Domain>
</CycloneDDS>
```

4. Add `export CYCLONEDDS_URI=file://${HOME}/cyclonedds_config.xml` at the bottom of `~/.bashrc` file
5. Build the code:
```
mkdir -p ~/delay_check_ws/src
cd ~/delay_check_ws/src
git clone https://github.com/mitsudome-r/pointcloud_delay_check.git
cd ~/delay_check_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Launching Nodes
`ros2 launch pointcloud_delay_check pointcloud_delay_check.launch.py use_multithread:=True use_intra_process:=True`
