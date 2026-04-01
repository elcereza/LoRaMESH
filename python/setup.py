from setuptools import setup, Extension, find_packages
import pybind11

ext_modules = [
    Extension(
        "loramesh.loramesh",
        [
            "src/loramesh/bindings.cpp",
            "src/loramesh/core/LoRaMESH.cpp",
            "src/loramesh/adapters/linux/LinuxSerialTransport.cpp",
        ],
        include_dirs=[
            pybind11.get_include(),
            "src/loramesh",
        ],
        language="c++",
        extra_compile_args=["-std=c++17"],
    ),
]

setup(
    name="loramesh",
    version="1.0.0",
    package_dir={"": "src"},
    packages=find_packages(where="src"),
    package_data={
        "loramesh": ["**/*.h", "**/*.cpp"],
    },
    include_package_data=True,
    ext_modules=ext_modules,
)