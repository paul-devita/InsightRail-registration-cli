# Configure the project
cmake -S . -B build `
    -G "Visual Studio 17 2022" `
    -A x64 `
    -DVCPKG_ROOT="C:/dev/vcpkg"

# Build Debug
cmake --build build --config Debug

# Run
.\build\bin\registration_cli.exe
