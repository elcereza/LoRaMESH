from setuptools import setup, Extension, find_packages
import pybind11
import sys

sources = [
    "src/loramesh/bindings.cpp",
    "src/loramesh/core/LoRaMESH.cpp",
]

extra_compile_args = []

if sys.platform.startswith("linux"):
    sources.append("src/loramesh/adapters/linux/LinuxSerialTransport.cpp")
    extra_compile_args = ["-std=c++17"]

elif sys.platform == "win32":
    sources.append("src/loramesh/adapters/windows/WindowsSerialTransport.cpp")
    extra_compile_args = ["/std:c++17"]

ext_modules = [
    Extension(
        "loramesh.loramesh",
        sources,
        include_dirs=[
            pybind11.get_include(),
            "src/loramesh",
        ],
        language="c++",
        extra_compile_args=extra_compile_args,
    ),
]

setup(
    name="loramesh",
    version="1.0.1",  # IMPORTANTE: subir versão
    package_dir={"": "src"},
    packages=find_packages(where="src"),
    package_data={
        "loramesh": ["**/*.h"],
    },
    include_package_data=True,
    ext_modules=ext_modules,
)