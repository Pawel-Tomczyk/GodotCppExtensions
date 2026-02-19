#pragma once

//c++ libraries
#include <queue>

//Godot libraries
#include <godot_cpp/classes/node.hpp>


class UartManager : public godot::Node{
    GDCLASS(UartManager, godot::Node)
public:
    int port;
    std::queue<int> msgQueue;
protected:
    static void _bind_methods();
private:
};
