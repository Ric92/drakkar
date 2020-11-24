# drakkar

# Usage

## 🚀&nbsp; Installation

* CMake > 3.10

```bash
git clone -b "release" "https://github.com/Kitware/CMake"
cd CMake
./bootstrap ; make ; sudo make install
```

* OpenCV > 3.2

```bash
sudo apt-get install -y libopencv-dev
```

* PCL

```bash
sudo apt-get install -y libpcl-dev
```

* RealSense2

```bash
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u

sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg
```
## 💚&nbsp; STL to PCD

* Convert yout STL file to PLY file with MeshLab.
* Convert the PLY file into PCD using pcl_ply2pcd input.ply output.pcd


## 📘&nbsp; License
The drakkar content management system is released under the under terms of the [MIT License](LICENSE).
