#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "core/LoRaMESH.h"
#include "adapters/linux/LinuxSerialTransport.h"

namespace py = pybind11;

class PyLoRaMESH
{
private:
    LinuxSerialTransport transport;
    LoRaMESH mesh;
    static void debugCallback(const char* msg)
    {
        py::print(msg);
    }

public:

    PyLoRaMESH(const std::string& device, int baud)
        : transport(device.c_str(), baud),
        mesh(&transport, &transport)
    {
    }

    void begin(bool debug=true, bool hex_dump=true)
    {
        if(debug)
        {
            mesh.setDebug(debugCallback, hex_dump);
        }

        mesh.begin(debug);
    }

    bool localRead()
    {
        return mesh.localRead();
    }

    bool setNetworkID(uint16_t id)
    {
        return mesh.setNetworkID(id);
    }

    bool setPassword(uint32_t password)
    {
        return mesh.setPassword(password);
    }

    bool getBPS()
    {
        return mesh.getBPS();
    }

    bool getClass()
    {
        return mesh.getClass();
    }

    bool setBPS(uint8_t bw, uint8_t sf, uint8_t cr)
    {
        return mesh.setBPS(bw, sf, cr);
    }

    bool setClass(uint8_t lora_class, uint8_t window)
    {
        return mesh.setClass(lora_class, window);
    }

    bool pinMode(uint16_t id, uint8_t gpio, uint8_t mode, uint8_t level=0)
    {
        return mesh.pinMode(id, gpio, mode, level);
    }

    bool digitalWrite(uint16_t id, uint8_t gpio, uint8_t val)
    {
        return mesh.digitalWrite(id, gpio, val);
    }

    uint8_t digitalRead(uint16_t id, uint8_t gpio)
    {
        return mesh.digitalRead(id, gpio);
    }

    uint16_t analogRead(uint16_t id, uint8_t gpio)
    {
        return mesh.analogRead(id, gpio);
    }

    int getNoise(uint16_t id, uint8_t select=1)
    {
        return mesh.getNoise(id, select);
    }

    int getR1(uint16_t rawADC, int R2)
    {
        return mesh.getR1(rawADC, R2);
    }

    double getTemp(uint16_t rawADC, int beta, int Rt=10000, int R2=10000)
    {
        return mesh.getTemp(rawADC, beta, Rt, R2);
    }

    uint16_t getLocalID()
    {
        return mesh.localId;
    }

    uint32_t getUniqueID()
    {
        return mesh.localUniqueId;
    }

    uint8_t getBW()
    {
        return mesh.BW;
    }

    uint8_t getSF()
    {
        return mesh.SF;
    }

    uint8_t getCR()
    {
        return mesh.CR;
    }

    uint8_t getClassValue()
    {
        return mesh.LoRa_class;
    }

    uint8_t getWindow()
    {
        return mesh.LoRa_window;
    }
};

PYBIND11_MODULE(loramesh, m)
{
    py::class_<PyLoRaMESH>(m, "LoRaMESH")
        .def(py::init<const std::string&, int>())
        .def("begin", &PyLoRaMESH::begin,
            py::arg("debug")=true,
            py::arg("hex_dump")=true)
        .def("localRead", &PyLoRaMESH::localRead)
        .def("setNetworkID", &PyLoRaMESH::setNetworkID)
        .def("setPassword", &PyLoRaMESH::setPassword)
        .def("getBPS", &PyLoRaMESH::getBPS)
        .def("getClass", &PyLoRaMESH::getClass)
        .def("setBPS", &PyLoRaMESH::setBPS)
        .def("setClass", &PyLoRaMESH::setClass)
        .def("pinMode", &PyLoRaMESH::pinMode)
        .def("digitalWrite", &PyLoRaMESH::digitalWrite)
        .def("digitalRead", &PyLoRaMESH::digitalRead)
        .def("analogRead", &PyLoRaMESH::analogRead)
        .def("getNoise", &PyLoRaMESH::getNoise)
        .def("getR1", &PyLoRaMESH::getR1)
        .def("getTemp", &PyLoRaMESH::getTemp)
        .def("getLocalID", &PyLoRaMESH::getLocalID)
        .def("getUniqueID", &PyLoRaMESH::getUniqueID)
        .def("getBW", &PyLoRaMESH::getBW)
        .def("getSF", &PyLoRaMESH::getSF)
        .def("getCR", &PyLoRaMESH::getCR)
        .def("getClassValue", &PyLoRaMESH::getClassValue)
        .def("getWindow", &PyLoRaMESH::getWindow);
}
