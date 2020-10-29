# CS301-Pacman
This project is to build on top of the given Pacman robot simulation library in order to implement a path finding algorithm to complete level 1 and 2 of the game. Pacman is a maze game, where the objective is to eat all the food items that are placed in the map.

In level 1 the entire map is filled with food items and the car must navigate the entire map to collect all these items. In level 2 the map is filled with five foods with their locations defined in the food_list array. The level is completed once all items are collected. For the car to be able to complete these levels a path finding algorithm must be used. In conjugation the car must be able to understand the path and move accordingly.

On the car simulation, there are seven light sensors that are used to detect black lines. Black lines on the map are considered a path and therefore the sensors light up on path detection. These sensors are useful for the cars movement as it allows it to always stay on track, as well as for intersection detection when the car needs to make a predetermined turn.
