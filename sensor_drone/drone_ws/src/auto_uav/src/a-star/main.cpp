#include <iostream>
#include "src/AStar.hpp"

int main()
{
    AStar::Generator generator;
    generator.setWorldSize({25, 25});
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);
    generator.addCollision({4,4});

    std::cout << "Generate path ... \n";
    auto path = generator.findPath({0, 0}, {20, 20});


    for(auto& coordinate : path) {
        std::cout << coordinate.x << " " << coordinate.y << "\n";
    }
}