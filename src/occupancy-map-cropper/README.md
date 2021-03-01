# Crop Map

This packages wraps a simple implementation of openCV library to crop an image, given a text file containing the vertices of the cropping polygon.

### Public Methods

* **bool** cropMap(std::string map_path, std::string polygon_path)

Method that provides main functionality of this library. Pass in the full path to map file in .pgm format and path to cropping polygon in .txt format to start cropping. 

Polygon text file should have at least 3 vertices and 1 robot position coordinate:

	0 0
	0 10
	10 10 
	10 0
	5 5

In the above example, the first 4 pairs of coordinates describe a square shaped polygon with side lengths of 10 units. The position of the robot is set to be (5,5). In the current implementation, robot position is not used for anything, but it must be defined in the polygon text file.

Save location of cropped image and the coordinates which describe the origin of the cropped image with respect to the original map can be set using the setter methods described below.

Returns success of cropping.

* **void** setCoordinatesSavePath(std::string path)

Sets the path and file name to save the origin information of the cropped map. Example path:

	/home/coordinates.txt

* **void** setCroppedSavePath(std::string path)

Sets the path and file name to save the cropped map. Example path:

	/home/cropped.png
	
* **void** setResolution(double res)

Set resolution of map that is being cropped. Unit in meters.

* **void** setMapOrigin(double origin_x, double origin_y)

Set the origin of original map that is being cropped. Origin points are in meters. 

**All of the above setter methods also have a corresponding getter method for public use**
