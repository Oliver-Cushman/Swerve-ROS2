#include "swerve_interfaces/msg/gamepad_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "SDL2/SDL.h"
#include <thread>

using namespace std::chrono_literals;

class InputNode : public rclcpp::Node
{
public:
    InputNode() : Node("input_node")
    {
        this->timer = this->create_wall_timer(20ms, std::bind(&InputNode::timer_callback, this));
        this->controller_publisher = this->create_publisher<swerve_interfaces::msg::GamepadState>("GamepadInput", 10);
        SDL_setenv("SDL_VIDEODRIVER", "dummy", 1);
        if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER | SDL_INIT_JOYSTICK) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "SDL_Init failed: %s", SDL_GetError());
        }
        RCLCPP_INFO(this->get_logger(), "SDL initialized with driver: %s", SDL_GetCurrentVideoDriver());
        this->gamepad = this->get_gamepad();
    }

private:
    SDL_GameController *get_gamepad()
    {
        SDL_GameController *attempt = nullptr;
        while (!attempt)
        {
            SDL_GameControllerEventState(SDL_ENABLE);
            SDL_JoystickEventState(SDL_ENABLE);
            SDL_GameControllerUpdate();
            SDL_JoystickUpdate();
            RCLCPP_INFO(this->get_logger(), "%i", SDL_NumJoysticks());
            for (int i = 0; i < SDL_NumJoysticks(); i++)
            {
                if (SDL_IsGameController(i))
                {
                    attempt = SDL_GameControllerOpen(i);
                }
                if (attempt)
                    return attempt;
            }
            RCLCPP_WARN(this->get_logger(), "Waiting for a controller...");
            std::this_thread::sleep_for(1s);
        }
        return attempt;
    }

    void timer_callback()
    {
    }
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<swerve_interfaces::msg::GamepadState>::SharedPtr controller_publisher;
    SDL_GameController *gamepad;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InputNode>());
    rclcpp::shutdown();
    SDL_QuitSubSystem(SDL_INIT_GAMECONTROLLER);
    return 0;
}