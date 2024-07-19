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

    if(!renderer)
    {
        RCLCPP_INFO(this->get_logger(), "Could not create SDL renderer");
    }

    if(!window)
    {
        RCLCPP_INFO(this->get_logger(), "Could not create SDL Window");
    }

    // Setup Platform/Renderer backends
    ImGui_ImplSDL2_InitForSDLRenderer(window.get(), renderer.get());
    ImGui_ImplSDLRenderer2_Init(renderer.get());

    RCLCPP_INFO(this->get_logger(), "Node %s fininshed initializing!",
                this->get_name());
}

bool Application::is_running() const
{
    return this->running;
}

void Application::handle_event(SDL_Event & e)
{
    // User requests quit
    if(e.type == SDL_QUIT)
    {
        running = false;
    }
}

void Application::update()
{
    if(!running)
    {
        return;
    }
    SDL_Event e;

    // Fetch next event or
    while(SDL_PollEvent(&e) == 1)
    {
        ImGui_ImplSDL2_ProcessEvent(&e);
        this->handle_event(e);
    }

    // Start the Dear ImGui frame
    ImGui_ImplSDLRenderer2_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
    {
        ImGui::Begin("Options"); // Create a window called "Hello, world!" and
                                 // append into it.
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                    1000.0f / io.Framerate, io.Framerate);
        ImGui::End();
    }

    // Rendering
    ImGui::Render();
    SDL_RenderSetScale(renderer.get(), io.DisplayFramebufferScale.x,
                       io.DisplayFramebufferScale.y);
    SDL_SetRenderDrawColor(renderer.get(), (Uint8)(clear_color.x * 255),
                           (Uint8)(clear_color.y * 255),
                           (Uint8)(clear_color.z * 255),
                           (Uint8)(clear_color.w * 255));
    SDL_RenderClear(renderer.get());

    // Initialize renderer color white for the background
    SDL_SetRenderDrawColor(renderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);

    // App Update

    // Update screen
    SDL_RenderPresent(renderer.get());
}