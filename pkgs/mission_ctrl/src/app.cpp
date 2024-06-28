#include "mission_ctrl/app.hpp"

Application::Application(int argc, char ** argv)
: rclcpp::Node("mission_ctrl_main")
, window(SDL_CreateWindow("Basic C++ SDL project", SDL_WINDOWPOS_UNDEFINED,
                          SDL_WINDOWPOS_UNDEFINED, WIDTH, HEIGHT,
                          SDL_WINDOW_SHOWN))
, renderer(SDL_CreateRenderer(window.get(), -1, SDL_RENDERER_ACCELERATED))
{
    (void)argc;
    (void)argv;
    RCLCPP_INFO(this->get_logger(), "Node %s fininshed initializing!", this->get_name());
}

bool Application::is_running() const
{
    return this->running;
}

void Application::update()
{
    SDL_Event e;

    // Wait indefinitely for the next available event
    SDL_WaitEvent(&e);

    // User requests quit
    if(e.type == SDL_QUIT)
    {
        running = false;
    }

    // Initialize renderer color white for the background
    SDL_SetRenderDrawColor(renderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);

    // App Update

    // Update screen
    SDL_RenderPresent(renderer.get());
}