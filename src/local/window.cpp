#include "window.hpp"

#include "backend/cpu.hpp"
#include "libs/imgui/imgui.h"
#include "libs/imgui/imgui_impl_sdl2.h"
#include "libs/imgui/imgui_impl_sdlrenderer2.h"


namespace
{
    constexpr float windowPixelScale = 2.5f;

    inline std::uint16_t rgb555(std::uint16_t pixel) noexcept
    {
        return (pixel << 3) | (pixel >> 2);
    }
}

Window::~Window()
{
    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    SDL_DestroyWindow(m_window);
    SDL_DestroyRenderer(m_renderer);
    SDL_Quit();
}

void Window::initialize(std::string_view gamePakTitle)
{
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) != 0)
    {
        throw std::runtime_error("failed to initialize SDL library");
    }

    m_window = SDL_CreateWindow(
        std::string(gamePakTitle).c_str(),
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        1000,
        650,
        SDL_WINDOW_SHOWN
    );

    if (m_window == nullptr)
    {
        throw std::runtime_error("failed to create window");
    }

    m_renderer = SDL_CreateRenderer(m_window, -1, 0);
    if (m_renderer == nullptr)
    {
        throw std::runtime_error("failed to create renderer");
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;

    ImGui_ImplSDL2_InitForSDLRenderer(m_window, m_renderer);
    ImGui_ImplSDLRenderer2_Init(m_renderer);
}

void Window::render(const std::uint16_t* framebuffer)
{
    constexpr ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoTitleBar |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoResize;

    ImGui::Begin("##GAME_WINDOW", nullptr, flags);

    // ImGui::ShowDemoWindow();

    ImVec2 windowSize = ImGui::GetWindowSize();
    ImDrawList* drawList = ImGui::GetWindowDrawList();

    float yOffset = ((windowSize.y - (160 * windowPixelScale)) / 2);
    float xOffset = ((windowSize.x - (240 * windowPixelScale)) / 2);

    for (int row = 0; row < 160; row++)
    {
        for (int col = 0; col < 240; col++)
        {
            auto yPos = (row * windowPixelScale) + yOffset;
            auto xPos = (col * windowPixelScale) + xOffset;
            std::uint16_t pixel = framebuffer[row * 240 + col];

            drawList->AddRectFilled(
                ImVec2(xPos, yPos),
                ImVec2(xPos + windowPixelScale, yPos + windowPixelScale),
                ImColor(
                    rgb555(pixel & 0x1F),
                    rgb555((pixel >> 5) & 0x1F),
                    rgb555((pixel >> 10) & 0x1F)
                )
            );
        }
    }

    ImGui::End();
}

void Window::run(std::string_view filepath)
{
    backend::CPU cpu(filepath);

    initialize(cpu.getGamePakTitle());

    bool shutdown = false;
    bool success = cpu.run([&](const std::uint16_t* framebuffer)
    {
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
            {
                shutdown = true;
                break;
            }
            ImGui_ImplSDL2_ProcessEvent(&event);
        }

        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        render(framebuffer);
        ImGui::Render();

        SDL_RenderClear(m_renderer);
        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), m_renderer);
        SDL_RenderPresent(m_renderer);
    }, shutdown);

    if (!success)
    {
        throw std::runtime_error("unexpected error has occured");
    }
}
