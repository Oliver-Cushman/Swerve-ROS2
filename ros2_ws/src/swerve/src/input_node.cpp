#include "swerve_interfaces/msg/gamepad_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "SDL2/SDL.h"
#include <thread>

using namespace std::chrono_literals;

class InputNode : public rclcpp::Node
{
public:
    InputNode() : Node("input")
    {
        this->timer = this->create_wall_timer(20ms, std::bind(&InputNode::timer_callback, this));
        this->input_publisher = this->create_publisher<swerve_interfaces::msg::GamepadState>("teleop/gamepad_input", 10);
        SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
        if (SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "SDL_Init failed: %s", SDL_GetError());
        }
        SDL_JoystickEventState(SDL_ENABLE);
        SDL_GameControllerEventState(SDL_ENABLE);
        this->gamepad = this->get_gamepad();
        if (!this->gamepad)
            throw std::runtime_error("Looks like we didn't get a controller...");
    }

private:
    void timer_callback()
    {
        auto gamepad_state = this->get_gamepad_state();
        this->input_publisher->publish(gamepad_state);
    }

    SDL_GameController *get_gamepad()
    {
        SDL_GameController *attempt = nullptr;
        while (!attempt)
        {
            SDL_GameControllerUpdate();
            SDL_JoystickUpdate();
            RCLCPP_INFO(this->get_logger(), "%i joystick(s) found", SDL_NumJoysticks());
            for (int i = 0; i < SDL_NumJoysticks(); i++)
            {
                if (SDL_IsGameController(i))
                    attempt = SDL_GameControllerOpen(i);
                if (attempt)
                    return attempt;
            }
            RCLCPP_WARN(this->get_logger(), "Waiting for a controller...");
            std::this_thread::sleep_for(1s);
        }
        return attempt;
    }

    float get_axis(SDL_GameControllerAxis axis)
    {
        return (float)SDL_GameControllerGetAxis(this->gamepad, axis) / this->GAMEPAD_AXIS_MAX;
    }

    bool get_btn(SDL_GameControllerButton btn)
    {
        return SDL_GameControllerGetButton(this->gamepad, btn) == 1;
    }

    swerve_interfaces::msg::GamepadState get_gamepad_state()
    {
        SDL_GameControllerUpdate();
        SDL_JoystickUpdate();
        auto gamepad_state = swerve_interfaces::msg::GamepadState();
        gamepad_state.left_x = this->get_axis(SDL_CONTROLLER_AXIS_LEFTX);
        gamepad_state.left_y = this->get_axis(SDL_CONTROLLER_AXIS_LEFTY) * -1;
        gamepad_state.right_x = this->get_axis(SDL_CONTROLLER_AXIS_RIGHTX);
        gamepad_state.right_y = this->get_axis(SDL_CONTROLLER_AXIS_RIGHTY) * -1;
        gamepad_state.left_trigger = this->get_axis(SDL_CONTROLLER_AXIS_TRIGGERLEFT);
        gamepad_state.right_trigger = this->get_axis(SDL_CONTROLLER_AXIS_TRIGGERRIGHT);
        gamepad_state.btn_north = this->get_btn(SDL_CONTROLLER_BUTTON_Y);
        gamepad_state.btn_south = this->get_btn(SDL_CONTROLLER_BUTTON_A);
        gamepad_state.btn_east = this->get_btn(SDL_CONTROLLER_BUTTON_B);
        gamepad_state.btn_west = this->get_btn(SDL_CONTROLLER_BUTTON_X);
        gamepad_state.left_bumper = this->get_btn(SDL_CONTROLLER_BUTTON_LEFTSHOULDER);
        gamepad_state.right_bumper = this->get_btn(SDL_CONTROLLER_BUTTON_RIGHTSHOULDER);
        gamepad_state.dpad_up = this->get_btn(SDL_CONTROLLER_BUTTON_DPAD_UP);
        gamepad_state.dpad_down = this->get_btn(SDL_CONTROLLER_BUTTON_DPAD_DOWN);
        gamepad_state.dpad_left = this->get_btn(SDL_CONTROLLER_BUTTON_DPAD_LEFT);
        gamepad_state.dpad_right = this->get_btn(SDL_CONTROLLER_BUTTON_DPAD_RIGHT);
        return gamepad_state;
    }

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<swerve_interfaces::msg::GamepadState>::SharedPtr input_publisher;
    SDL_GameController *gamepad;

    const float GAMEPAD_AXIS_MAX = 32768.0;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InputNode>());
    rclcpp::shutdown();
    SDL_QuitSubSystem(SDL_INIT_GAMECONTROLLER);
    return 0;
}