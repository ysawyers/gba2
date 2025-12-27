#include <SDL.h>
#include <string_view>


class Window
{
public:
    Window() = default;
    Window(Window&) = delete;
    Window& operator=(Window&) = delete;
    Window(Window&&) = delete;
    Window& operator=(Window&&) = delete;
    ~Window();

    /*!
        \brief Blocks until window is closed

        \param[in] filepath Path to ROM file
    */
    void run(std::string_view filepath);

private:
    void initialize(std::string_view title);

    void render(const std::uint16_t* framebuffer);

private:
    SDL_Window* m_window{nullptr};

    SDL_Renderer* m_renderer{nullptr};
};
