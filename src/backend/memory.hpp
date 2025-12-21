#include <string_view>
#include <fstream>
#include <vector>


namespace backend
{
    class Memory
    {
    public:
        /*!
            /param[in] filepath Path to ROM file
        */
        Memory(std::string_view filepath)
        {
            std::ifstream file(filepath, std::ios::binary);
            auto len = file.tellg();

            m_rom.reserve(len);
            file.read(std::bit_cast<char*>(m_rom.data()), len);
        }

        void read();

        void write();

    private:
        std::vector<std::uint8_t> m_rom;
    };
}
