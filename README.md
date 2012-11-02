astar
=====

A star search algorithm using different heuristics

The program runs the same A* search function with arguments as the startA and startB points and the type of heuristic. Type 1, 2 and 3 correspond to euclidean, manhattan and chessboard heuristics. The updatepriority function takes this type as argument and updates h(x) as the value determined by the heuristic function. The map is a 2D array of 0s and 1s specify obstacles.