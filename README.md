# Pathfinding with Moving Obstacles and Terrain Constraint

It's a C++ implementation of A* search algorithm to find the shortest path among obstacles.

The A* search is further improved to support moving obstacles and terrain height.

## Approach
#### Map information
A matrix of integer will be created to represent map information (wall, obstacles, terrain height... etc). This map information can be saved and read from local .txt files.

Given a origin coordinate and a destination coordinate, the shortest path can be found using improved A* search.
#### Moving direction
Currently, the possible moving directions from a given position are the eight directions around it, which means the possible positions for the next step are adjacent to that position. A path needs to go to at least one of these eight positions in order to move forward.
#### Wall
A wall is treated as a position which a path can't go through. 
#### Terrain height
Terrain height is handled by checking if the next position is too low/too high (can be adjusted). If it's not too steep, a path can go through.
#### Moving obstacle
Moving obstacle is handled by treating it as another object which uses the same algorithm to find path. Whenever a object is finding a path on the map, the occupation of that object and the timing will be recorded, and this information is then shared to all objects to prevent collisions between objects.

If the next position is occupied by a moving obstacles (treated differently from wall/height), the path will "wait" and let the blocking object go first, and check where to go at next moment. 
## A* search
#### Heuristic function
Currently, there are three choices for heuristic function used in A* search algorithm:
`Diagonal, Manhattan, Euclidean`

Those are 'enum' type and can be set by using `setHType()` function.

#### Map constraint type
Currently, there are three choices for constraint type for map information:
`Wall, Height, None`

Those are 'enum' type and can be set by using `setCType()` function.

- Height: Handle height as described above.
- Wall: Simply treat every height on the map as wall.
- None: Treat the whole map as flat ground and you can go everywhere.

## Other information
The X and Y direction is rotated 90 degree clockwise in the implementation. Keep this in mind if you want to see the drew path.

## Example

    AStar road(50, 60, 10, -2, 50);
    road.setHType(Euclidean);
    road.setCType(None);
    
    road.createMapInfo();

    //get mapInfo from the file and display it
    if ( road.extractMapInfo() ){
        road.showMapInfo();
    }

    //If a path from (5, 47) to (38, 3) can be found, draw the path
    if (road.aStarSearchMoving(5, 47, 38, 3)){
        road.drawLineObstacle();
    }
    //If a path from (11, 9) to (43, 47) can be found, print out the path coordinates and draw the path
    if (road.aStarSearchMoving(11, 9, 43, 47)){
        road.showPath();
        road.drawLineObstacle();
    }
