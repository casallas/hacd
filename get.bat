cd hacd
cd include
copy \p4\sw\physx\APEXSDK\1.2\feature\DynamicSystem\shared\general\hacd\include\*.h
cd ..
cd public
copy PlatformConfigHACD.h temp.h
copy \p4\sw\physx\APEXSDK\1.2\feature\DynamicSystem\shared\general\hacd\public\*.h
copy temp.h PlatformConfigHACD.h
del temp.h
cd ..
cd src
copy \p4\sw\physx\APEXSDK\1.2\feature\DynamicSystem\shared\general\hacd\src\*.cpp
cd ..
cd ..
