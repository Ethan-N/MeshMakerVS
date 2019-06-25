# MeshMakerVS
Uses OpenFrameworks to create a mesh using Structure Core Depth and RGB Data. Will be used for occlusion for AR.

To Get Structure Core working with Visual Studio and OpenFrameworks:
Add Structure.dll to Win32 System Folder

Under Visual Studio Project Preferences:
Add headers Folder from Occipital SDK to Additional Include Directories (C/C++ General)
Add path of Structure.lib and Structure.dll to Additional Library Directories (Linker)
Add path of Structure.lib to Additional Dependencies (Linker)
