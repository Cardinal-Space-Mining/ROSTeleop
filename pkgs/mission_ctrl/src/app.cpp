#include "mission_ctrl/app.hpp"

const std::array<std::string, 5> RobotTeleopInterface::motors = {
    {"track_right", "track_left", "trencher", "hopper_belt",
     "hopper_actuator"}};

RobotTeleopInterface::RobotTeleopInterface(rclcpp::Node & parent)
: robot_state(motors.size())
, joy_sub(parent.create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      [this](const sensor_msgs::msg::Joy & joy) { this->joy = joy; }))
, teleop_update_timer(parent.create_wall_timer(
      100ms, std::bind(&RobotTeleopInterface::update_motors, this)))

{

    for(size_t i = 0; i < motors.size(); i++)
    {
        auto & motor = motors[i];
        talon_ctrl_pubs.emplace_back(
            parent.create_publisher<custom_types::msg::TalonCtrl>(
                motor + "_ctrl", 10));
        talon_info_subs.emplace_back(
            parent.create_subscription<custom_types::msg::TalonInfo>(
                motor + "_info", 10,
                [this, i](const custom_types::msg::TalonInfo & info)
                { this->robot_state[i] = info; }));
    }
}

void RobotTeleopInterface::update_motors()
{
    auto motor_settings =
        this->teleop_state.update(this->robot_state, this->joy);
    for(size_t i = 0; i < motor_settings.size(); i++)
    {
        talon_ctrl_pubs[i]->publish(motor_settings[i]);
    }
}

Application::Application(int argc, char ** argv)
: rclcpp::Node("mission_ctrl_main")
, window(SDL_CreateWindow("Basic C++ SDL project", SDL_WINDOWPOS_UNDEFINED,
                          SDL_WINDOWPOS_UNDEFINED, WIDTH, HEIGHT,
                          SDL_WINDOW_SHOWN))
, renderer(SDL_CreateRenderer(window.get(), -1, SDL_RENDERER_ACCELERATED))
, track_right_pub(
      create_publisher<custom_types::msg::TalonCtrl>("track_right", 10))
, heartbeat(create_publisher<std_msgs::msg::Int32>("heartbeat", 10))
, heartbeat_timer(this->create_wall_timer(BEAT_TIME,
                                          [this]()
                                          {
                                              if(this->bot_enabled)
                                              {
                                                  std_msgs::msg::Int32 msg;
                                                  msg.data =
                                                      ENABLE_TIME.count();
                                                  this->heartbeat->publish(msg);
                                              }
                                          }))
, teleop_interface(*this)

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

    frame_timer =
        this->create_wall_timer(frame_time, [this]() { this->update(); });

    RCLCPP_INFO(this->get_logger(), "Node %s fininshed initializing!",
                this->get_name());
}

void Application::handle_event(SDL_Event & e)
{
    // User requests quit
    if(e.type == SDL_QUIT)
    {
        rclcpp::shutdown();
    }
}

void Application::update_motors()
{
    custom_types::msg::TalonCtrl msg;
    msg.mode = msg.PERCENT_OUTPUT;
    msg.value = this->track_right_velo;
    this->track_right_pub->publish(msg);
}

void Application::update()
{
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

        ImGui::Checkbox("Enable Robot", &this->bot_enabled);

        ImGui::SliderFloat("RightTrack", &this->track_right_velo, -1, 1);
        ImGui::End();
    }

    // Rendering
    ImGui::Render();
    SDL_RenderSetScale(renderer.get(), io.DisplayFramebufferScale.x,
                       io.DisplayFramebufferScale.y);
    SDL_SetRenderDrawColor(renderer.get(), reset_color[0], reset_color[1],
                           reset_color[2], reset_color[3]);
    SDL_RenderClear(renderer.get());

    // App Update

    ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer.get());
    SDL_RenderPresent(renderer.get());

    // Update screen
    SDL_RenderPresent(renderer.get());

    this->update_motors();
}