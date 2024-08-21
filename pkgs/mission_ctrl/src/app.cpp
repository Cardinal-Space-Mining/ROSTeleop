#include "mission_ctrl/app.hpp"

RobotTeleopInterface::RobotTeleopInterface(rclcpp::Node & parent)
: joy_sub(parent.create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      [this](const sensor_msgs::msg::Joy & joy) { this->joy = joy; }))

, right_track_ctrl(talon_ctrl_pub(parent, "track_right_ctrl"))
, right_track_info(parent.create_subscription<custom_types::msg::TalonInfo>(
      "track_right_info", 10, [this](const custom_types::msg::TalonInfo & msg)
      { this->robot_state.track_right = msg; }))
, left_track_ctrl(talon_ctrl_pub(parent, "track_left_ctrl"))
, left_track_info(parent.create_subscription<custom_types::msg::TalonInfo>(
      "track_left_info", 10, [this](const custom_types::msg::TalonInfo & msg)
      { this->robot_state.track_left = msg; }))
, trencher_ctrl(talon_ctrl_pub(parent, "trencher_ctrl"))
, trencher_info(parent.create_subscription<custom_types::msg::TalonInfo>(
      "trencher_info", 10, [this](const custom_types::msg::TalonInfo & msg)
      { this->robot_state.trencher = msg; }))

, hopper_belt_ctrl(talon_ctrl_pub(parent, "hopper_belt_ctrl"))
, hopper_belt_info(parent.create_subscription<custom_types::msg::TalonInfo>(
      "hopper_belt_info", 10, [this](const custom_types::msg::TalonInfo & msg)
      { this->robot_state.hopper_belt = msg; }))

, hopper_actuator_ctrl(talon_ctrl_pub(parent, "hopper_actuator_ctrl"))
, hopper_actuator_info(parent.create_subscription<custom_types::msg::TalonInfo>(
      "hopper_actuator_info", 10,
      [this](const custom_types::msg::TalonInfo & msg)
      { this->robot_state.hopper_actuator = msg; }))

, teleop_update_timer(parent.create_wall_timer(
      100ms, [this](){this->update_motors();}))

{
}

std::shared_ptr<rclcpp::Publisher<custom_types::msg::TalonCtrl>>
RobotTeleopInterface::talon_ctrl_pub(rclcpp::Node & parent,
                                     const std::string & name)
{
    return parent.create_publisher<custom_types::msg::TalonCtrl>(name, 10);
}

void RobotTeleopInterface::update_motors()
{
    MotorSettings motor_settings =
        this->teleop_state.update(this->robot_state, this->joy);
    right_track_ctrl->publish(motor_settings.track_right);
    left_track_ctrl->publish(motor_settings.track_left);
    trencher_ctrl->publish(motor_settings.trencher);
    hopper_belt_ctrl->publish(motor_settings.hopper_belt);
    hopper_actuator_ctrl->publish(motor_settings.hopper_actuator);
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