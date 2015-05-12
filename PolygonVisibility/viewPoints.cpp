/* view.c
 
 Laura Toma, Ivy Xing, Zackery Leman
 
 What it does:
 
 
 */
#include <queue>
#include <vector>
#include "geom.h"
#include "rtimer.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

using namespace std;

GLfloat red[3] = {1.0, 0.0, 0.0};
GLfloat green[3] = {0.0, 1.0, 0.0};
GLfloat blue[3] = {0.0, 0.0, 1.0};
GLfloat black[3] = {0.0, 0.0, 0.0};
GLfloat white[3] = {1.0, 1.0, 1.0};
GLfloat gray[3] = {0.5, 0.5, 0.5};
GLfloat yellow[3] = {1.0, 1.0, 0.0};
GLfloat orange[3] = {1.0, 0.5, 0.0};
GLfloat magenta[3] = {1.0, 0.0, 1.0};
GLfloat cyan[3] = {0.0, 1.0, 1.0};

GLint fillmode = 0;

enum Direction {UP, DOWN, LEFT, RIGHT};

/**** Forward declarations of functions ****/
/* Display and responsive functions */
void display(void);
void keypress(unsigned char key, int x, int y);
void mouse(int button, int state, int Mx, int My);
void drag(int x, int y);
void startMoving(void);
int move(Direction d, double offset);
int canMove(Direction d);
int hitWall(point2D p, Direction d, double radius);
/* Drawing functions */
void drawCircle(float cx, float cy, float r, int num_segments);
void drawCirclesAroundVertices(void);
void drawSegment(segment2D s);
void drawPolygon(void);
void fillTraingle(point2D a, point2D b, point2D c);
void erase(void);
/* Helper computation functions */
int intersectPolygon(segment2D current);
int insidePolygon(point2D p);
void computeSlopes(void);
void clearVectors(void);
point2D closestEdgePoint(segment2D line, vector<int> edgeIndices);
/* Main computation functions */
void computeVisibleArea(void);
// Helper function to computeVisibleArea(void)
void computeVisibleVertices(void);
// Helper function to computeVisibleArea(void)
void extendVisibleVertices(void);
// Helper function to extendVisibleVertices(void)
void refineVisibleVerticesExtensions(segment2D extendedLine, int index);
// Helper function to computeVisibleArea(void)
void connectExtendedLineIntersections(void);

/**** Global variables ****/
int hasFinishedPolygon = 0; // Whether the user has finished drawing the polygon.
point2D visiblityPoint; // The point of visibility inside the polygon.
point2D firstClick = {0, 0};
point2D secondClick = {0, 0};

const int WINDOWSIZE = 500;
const int NUM_SEGMENTS = 800; // Number of segments that form the circle.
const double OFFSET = 1; // Number of pixels the point moves.

//Priority queue comparison function
class CompareStateDistance {
public:
    bool operator()(State& t1, State& t2)
    {
        if (t1.distance > t2.distance) return true;
        return false;
    }
};



//NOTE: all the structures below need to be global so that they can be rendered
// The array of segments that form the polygon.
vector<vector<segment2D>>  polygons;
vector<segment2D>  polygonSegments;
priority_queue<State,vector<State>,CompareStateDistance> nextStateQueue;


// The vertices of the polygon.
vector<point2D> polygonVertices;
// The array of segments that form the visible area of the polygon.
vector<segment2D>  visibleAreaSegments;
// The slope of the visible area segments.
vector<double>visibleAreaSegmentSlopes;
// The vertices of the polygon that are visible from the visibility point.
vector<point2D> visibleAreaVertices;
// The intersection points of the extended lines with the egdes of the polygon.
vector<point2D> extendedLineIntersections;

//Check if any of the four sides of the rectangular (square) robot intersects any of the edges of the obstacles
bool robotHitsObstacle(double robotCenterX, double robotCenterY ){
    
    
    return true;
}

//Adds all viable sucessor states to current state to the priority queue
void generateSuccessors(State robotState){
    
    point2D robotCenter = robotState.location;
    
    //Up
    if (!robotHitsObstacle(robotCenter.x, robotCenter.y + 1)) {
        
        State * nextPossibleState = new State();
        nextPossibleState->location = {robotCenter.x, robotCenter.y + 1};
        nextPossibleState->path = robotState.path;
        nextPossibleState->path.push_back(nextPossibleState->location );
        nextPossibleState->distance = distance_(nextPossibleState->location,secondClick) + nextPossibleState->path.size();
        nextStateQueue.push(*nextPossibleState);
        
    }
    //Down
    if (!robotHitsObstacle(robotCenter.x, robotCenter.y - 1)) {
        
        State * nextPossibleState = new State();
        nextPossibleState->location = {robotCenter.x, robotCenter.y - 1};
        nextPossibleState->distance = distance_(nextPossibleState->location,secondClick) + nextPossibleState->path.size();
        nextPossibleState->path = robotState.path;
        nextPossibleState->path.push_back(nextPossibleState->location );
        nextStateQueue.push(*nextPossibleState);
    }
    
    //Right
    if (!robotHitsObstacle(robotCenter.x + 1, robotCenter.y)) {
        
        State * nextPossibleState = new State();
        nextPossibleState->location = {robotCenter.x + 1, robotCenter.y};
        nextPossibleState->distance = distance_(nextPossibleState->location,secondClick) + nextPossibleState->path.size();
        nextPossibleState->path = robotState.path;
        nextPossibleState->path.push_back(nextPossibleState->location);
        nextStateQueue.push(*nextPossibleState);
    }
    
    //Left
    if (!robotHitsObstacle(robotCenter.x - 1, robotCenter.y)) {
        
        State * nextPossibleState = new State();
        nextPossibleState->location = {robotCenter.x - 1, robotCenter.y};
        nextPossibleState->distance = distance_(nextPossibleState->location,secondClick) + nextPossibleState->path.size();
        nextPossibleState->path = robotState.path;
        nextPossibleState->path.push_back(nextPossibleState->location );
        nextStateQueue.push(*nextPossibleState);
    }
    
}

//Called to find the shortest path around the obstacles. Returns this path.
vector<point2D>  findShortestPath(){
    
    double startingX = 4;
    double startingY = 4;
    
    generateSuccessorsOfState(currentState);
    
    double nextPossibleState->path.size() = 0;
    State * nextPossibleState = new State();
    nextPossibleState->location = {x, y};
    nextPossibleState->distance = distance_(nextPossibleState->location,secondClick) + nextPossibleState->path.size();
    nextStateQueue.push(*nextPossibleState);
    
}





/* ****************************** */
int main(int argc, char** argv) {
    
    
    findShortestPath();
    
    
    
    
    State test =  nextStateQueue.top();
    nextStateQueue.pop();
    
    /* initialize GLUT  */
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
    glutInitWindowSize(WINDOWSIZE, WINDOWSIZE);
    glutInitWindowPosition(100,100);
    glutCreateWindow(argv[0]);
    
    /* register callback functions */
    glutDisplayFunc(display);
    glutKeyboardFunc(keypress);
    glutMouseFunc(mouse);
    glutMotionFunc(drag);
    
    /* init GL */
    /* set background color black*/
    glClearColor(0.3,0.3, 0.3, 0.3);
    /* here we can enable depth testing and double buffering and so
     on */
    
    /* give control to event handler */
    glutMainLoop();
    
    
    
    
    
    
    return 0;
}

/**** Drawing functions ****/

// Draw a line segment.
void drawSegment(segment2D s) {
    glBegin(GL_LINES);
    glVertex2f(s.start.x, s.start.y);
    glVertex2f(s.end.x, s.end.y);
    glEnd();
}

// Generic openGL draw circle method.
void drawCircle(float cx, float cy, float r, int num_segments, int filled) {
    if (filled) {
        float x2, y2;
        float angle;
        
        glBegin(GL_TRIANGLE_FAN);
        glVertex2f(cx, cy);
        for (angle = 1.0f; angle < 361.0f; angle += 0.2) {
            x2 = cx + sin(angle) * r;
            y2 = cy + cos(angle) * r;
            glVertex2f(x2, y2);
        }
        glEnd();
    } else {
        float theta = 2 * 3.1415926 / float(num_segments);
        float c = cosf(theta); //precalculate the sine and cosine
        float s = sinf(theta);
        float t;
        float x = r; //we start at angle = 0
        float y = 0;
        
        glBegin(GL_LINE_LOOP);
        for(int ii = 0; ii < num_segments; ii++) {
            glVertex2f(x + cx, y + cy);//output vertex
            //apply the rotation matrix
            t = x;
            x = c * x - s * y;
            y = s * t + c * y;
        }
        glEnd();
    }
}

// Draw circles around each vertex to make it more visible.
void drawCirclesAroundVertices(void) {
    for (int i = 0; i < polygonVertices.size(); i++) {
        point2D v = polygonVertices[i];
        drawCircle(v.x, v.y, RADIUS, NUM_SEGMENTS, 0);
    }
}

// Draw the shape of the polygon.
void drawPolygon(void) {
    for (int i = 0; i < polygonSegments.size(); i++) {
        drawSegment(polygonSegments[i]);
    }
}

// Draw the point of visibility inside the polygon as a filled circle.
void drawVisbilityPoint(void) {
    drawCircle(visiblityPoint.x, visiblityPoint.y, RADIUS, NUM_SEGMENTS, 1);
}

// Draw line segments to the visible verticies.
void drawVisibleAreaSegments(void) {
    for (int i = 0; i < visibleAreaSegments.size(); i++) {
        drawSegment(visibleAreaSegments[i]);
    }
}

// Fill a triangle with the given coordinates of vertices.
void fillTraingle(point2D a, point2D b, point2D c) {
    glBegin(GL_TRIANGLES);
    glVertex2f(a.x, a.y);  //bottom left vertex
    glVertex2f(b.x, b.y);  //top vertex
    glVertex2f(c.x, c.y);  //bottom right vertex
    glEnd();
}

// Erase the polygon.
void erase(void) {
    polygonSegments.clear();
    polygonVertices.clear();
    visiblityPoint = {-1, -1};
    
    // Clear computations.
    clearVectors();
    // Allow user to draw a new polygon.
    hasFinishedPolygon = 0;
    // Update dispplay.
    glutPostRedisplay();
}

/**** Computation Functions ****/

// Return 1 if current edge intersects with any other edge of the polygon;
// 0 otherwise.
int intersectPolygon(segment2D current) {
    if (polygonSegments.size() > 0) {
        // The last segment is the current segment.
        for (int i = 0; i < polygonSegments.size() - 1; i++) {
            if (intersect(current, polygonSegments[i])) {
                return 1;
            }
        }
    }
    return  0;
}

// Return 1 if the visibility point is inside the polygon; 0 otherwise.
int insidePolygon(point2D p) {
    // Draw lines starting from the visibility point parallel to the x-axis.
    segment2D testLineRight, testLineLeft;
    testLineRight.start = p;
    testLineRight.end = {WINDOWSIZE * 2, p.y };
    testLineLeft.start = p;
    testLineLeft.end  = {-1 * WINDOWSIZE, p.y};
    
    // Count the number of intersections with the polygon.
    int numIntersectionsRight = 0, numIntersectionsLeft = 0;
    for (int i = 0; i < polygonSegments.size(); i++) {
        if (intersect_proper(testLineRight, polygonSegments[i])) {
            numIntersectionsRight++;
        }
        if (intersect_proper(testLineLeft, polygonSegments[i])) {
            numIntersectionsLeft++;
        }
    }
    // An odd number of intersections means the point is insde.
    if (numIntersectionsRight % 2 == 1 ||
        numIntersectionsLeft % 2 == 1) {
        return 1;
    }
    return 0;
}

// Compute and store the slopes of the visible area segments.
void computeSlopes(void) {
    for (int i = 0; i < visibleAreaSegments.size(); i++) {
        double m = slope(visibleAreaSegments[i]);
        visibleAreaSegmentSlopes.push_back(m);
    }
}

// Compute the point of intersection of the edge closest to visibility point.
point2D closestEdgePoint(segment2D line, vector<int> edgeIndices) {
    double closestDistance = INFINITY;
    point2D closestIntersection;
    for (int i = 0; i < edgeIndices.size(); i++) {
        
        // Compute the intersection points and compare distance.
        point2D intersection = computeIntersection(line, polygonSegments[edgeIndices[i]]);
        double d = distance_(visiblityPoint, intersection);
        
        if (d < closestDistance) {
            closestDistance = d;
            closestIntersection = intersection;
        }
    }
    return closestIntersection;
}

// Compute the initial visible vertices from the visibility point.
void computeVisibleVertices(void) {
    for (int i = 0; i < polygonVertices.size(); i++) {
        // Create line segments from visibility point to other vertices.
        segment2D line = {visiblityPoint, polygonVertices[i]};
        
        // Check if this segment intersects any polygon edge.
        int hasIntersected = 0;
        for (int j = 0; j < polygonSegments.size(); j++) {
            if (intersect_proper(line, polygonSegments[j])) {
                hasIntersected = 1;
            }
        }
        // If no intersection, add vertex and segment to the vectors to be drawn.
        if (!hasIntersected) {
            visibleAreaSegments.push_back(line);
            visibleAreaVertices.push_back(line.end);
        }
    }
}

// Extend visiblility lines of certain vertices to the edge of the polygon.
void extendVisibleVertices(void) {
    for (int i = 0; i < visibleAreaVertices.size(); i++) {
        point2D p = visibleAreaVertices[i];
        
        // Endpoint of the extension line.
        point2D offScreenEndPoint;
        double xDiff = p.x - visiblityPoint.x;
        
        if (xDiff > 0) {
            // Draw extension segment to the right of visibility point.
            offScreenEndPoint.x = visiblityPoint.x + WINDOWSIZE;
            offScreenEndPoint.y = WINDOWSIZE * visibleAreaSegmentSlopes[i] + visiblityPoint.y;
        } else if (xDiff < 0){
            // Draw extension segment to the left of visibility point.
            offScreenEndPoint.x = visiblityPoint.x - WINDOWSIZE;
            offScreenEndPoint.y = -1 * WINDOWSIZE * visibleAreaSegmentSlopes[i] + visiblityPoint.y;
        } else {
            // Vertical line.
            offScreenEndPoint.x = visiblityPoint.x;
            if (p.y - visiblityPoint.y > 0) {
                // Draw extension segment above visibility point.
                offScreenEndPoint.y = WINDOWSIZE;
            } else {
                // Draw extension segment below visibility point.
                offScreenEndPoint.y = 0;
            }
        }
        segment2D extendedLine = {visiblityPoint, offScreenEndPoint};
        
        // Cut off the extension so that line remains inside polygon.
        refineVisibleVerticesExtensions(extendedLine, i);
    }
}

// Helper function to extend lines from visible verticies.
void refineVisibleVerticesExtensions(segment2D extendedLine, int index) {
    // Check the number of proper intersections between the long extended line and the polygon.
    int numProperIntersections = 0;
    vector<int> indicesOfIntersectingEdges;
    
    for (int i = 0; i < polygonSegments.size(); i++) {
        if (intersect_proper(extendedLine, polygonSegments[i])) {
            numProperIntersections++;
            indicesOfIntersectingEdges.push_back(i);
        }
    }
    // Extend this segment to the edge if an odd number of proper intersections was found.
    if (numProperIntersections % 2 == 1) {
        point2D edgePoint = closestEdgePoint(extendedLine, indicesOfIntersectingEdges);
        extendedLine.end = edgePoint;
        // Add to the vertices and segments to be drawn.
        extendedLineIntersections.push_back(edgePoint);
        visibleAreaSegments[index] = extendedLine;
    }
}

// Connect the recomputed visible verticies.
void connectExtendedLineIntersections(void) {
    // Add all the extended lines' endpoints.
    for (int i = 0; i < extendedLineIntersections.size(); i++) {
        visibleAreaVertices.push_back(extendedLineIntersections[i]);
    }
    
    // Iterate through the vertices of the visible area of the polygon.
    for (int i = 0; i < visibleAreaVertices.size() - 1; i++) {
        for (int j = i + 1; j < visibleAreaVertices.size(); j++) {
            segment2D line = {visibleAreaVertices[i], visibleAreaVertices[j]};
            
            for (int s = 0; s < polygonSegments.size(); s++) {
                segment2D edge = polygonSegments[s];
                
                // If the visibility verticies are along an edge, connect them.
                if (collinear(edge.start, edge.end, line.start) &&
                    collinear(edge.start, edge.end, line.end) &&
                    between(edge.start, edge.end, line.start) &&
                    between(edge.start, edge.end, line.end)) {
                    visibleAreaSegments.push_back(line);
                }
            }
        }
    }
}

// Call various functions to compute the visible area of the polygon.
void computeVisibleArea(void) {
    // Clear previous computation, if any.
    clearVectors();
    // New computation.
    computeVisibleVertices();
    computeSlopes();
    extendVisibleVertices();
    connectExtendedLineIntersections();
}

// Clear all stored data; called when visibility point changes.
void clearVectors(void) {
    visibleAreaVertices.clear();
    visibleAreaSegments.clear();
    visibleAreaSegmentSlopes.clear();
    extendedLineIntersections.clear();
}

/**** Responsive and Display Functions ****/

// Respond to mouse clicks.
void mouse(int button, int state, int Mx, int My) {
    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        point2D mouseClick = {(double)Mx, (double)(WINDOWSIZE - My)};
        
        // User finished drawing polygon, so compute visibility area.
        if (hasFinishedPolygon) {
            visiblityPoint = mouseClick;
            
            if (insidePolygon(visiblityPoint)) {
                computeVisibleArea();
                
                // Store the first click of the visibility point.
                if (firstClick.x == 0 && firstClick.y == 0) {
                    firstClick = visiblityPoint;
                }
            } else {
                printf("Please click inside the polygon.");
            }
            // User is currently drawing polygon.
        } else {
            // Check if this is the first point drawn on canvas - if not, draw a line.
            if (polygonVertices.size() > 0) {
                // If user clicks back to the first point, he/she is finishing the polygon.
                point2D firstPoint = polygonVertices[0];
                if (withinRadius(mouseClick, firstPoint) && polygonSegments.size() > 2) {
                    mouseClick = firstPoint;
                    hasFinishedPolygon = 1;
                }
                
                
                // Last element in the vector is the previous point.
                segment2D currentLine = {polygonVertices.back(), mouseClick};
                // If line doesn't intersect other edges or if it's the last line.
                if (!intersectPolygon(currentLine) || hasFinishedPolygon) {
                    // Add new line.
                    polygonSegments.push_back(currentLine);
                    if (!hasFinishedPolygon) {
                        // Add new vertex.
                        polygonVertices.push_back(mouseClick);
                    }
                } else {
                    printf("Polygon cannot intersect itself.");
                }
            } else {
                // Add first point drawn on canvas.
                polygonVertices.push_back(mouseClick);
            }
        }
        // Update display.
        glutPostRedisplay();
    }
}

// Responds to mouse drag (of the visibility point).
void drag(int x, int y) {
    if (hasFinishedPolygon && insidePolygon(visiblityPoint)) {
        // Update visibility point.
        visiblityPoint = {(double)x, (double)(WINDOWSIZE - y)};
        // Re-compute.
        computeVisibleArea();
        // Update display.
        glutPostRedisplay();
    }
}

// Move the visibility point in the desired direction.
// Return 1 if the direction is valid (new point inside polygon)
// Return 0 if the point hits a wall.
int move(Direction d, double offset) {
    point2D p;
    switch (d) {
        case UP:
            p = {visiblityPoint.x, visiblityPoint.y + offset};
            break;
        case DOWN:
            p = {visiblityPoint.x, visiblityPoint.y - offset};
            break;
        case LEFT:
            p = {visiblityPoint.x - offset, visiblityPoint.y};
            break;
        case RIGHT:
            p = {visiblityPoint.x + offset, visiblityPoint.y};
            break;
        default:
            break;
    }
    
    int isInside = 0;
    // Move the visibility point and compute area.
    if (!hitWall(p, d, RADIUS)) {
        visiblityPoint = p;
        isInside = 1;
        computeVisibleArea();
        display();
    }
    return isInside;
}

// Check if there is room to move.
int canMove(Direction d) {
    return move(d, OFFSET);
}

// Whether the point has hit a wall.
int hitWall(point2D p, Direction d, double radius) {
    segment2D s;
    point2D start, end;
    if (d == LEFT || d == RIGHT) {
        start = {p.x - radius, p.y};
        end = {p.x + radius, p.y};
    } else {
        start = {p.x, p.y + radius};
        end = {p.x, p.y - radius};
    }
    s = {start, end};
    
    for (int i = 0; i < polygonSegments.size(); i++) {
        if (intersect(s, polygonSegments[i])) {
            return 1;
        }
    }
    return 0;
}

// Automatically move the visibility point.
void startMoving(void) {
    static int lastFrameTime = 0;
    int now, elapsed_ms;
    Direction dir = LEFT;
    int hit_x = 0, hit_y = 0;
    
    while (1) {
        now = glutGet (GLUT_ELAPSED_TIME);
        elapsed_ms = now - lastFrameTime;
        
        // Move visibility point every time interval.
        if (elapsed_ms  > 1) {
            // Double check if the point is inside the polygon.
            if (!insidePolygon(visiblityPoint)) {
                // If not, reset point to default position.
                visiblityPoint = firstClick;
                hit_x = 0;
                hit_y = 0;
                
                printf("HERE");
            }
            
            // If hitting the wall horizontally, move it vertically, and vice versa.
            if (hit_x) {
                // Check if there is room to move up.
                dir = canMove(DOWN) ? DOWN : UP;
                hit_x = 0;
                hit_y = 0;
            } else if (hit_y) {
                // Check if there is room to move left.
                dir = canMove(LEFT) ? LEFT : RIGHT;
                hit_y = 0;
                hit_y = 0;
            }
            
            // Has hit a wall.
            if (!move(dir, OFFSET)) {
                if (dir == LEFT || dir == RIGHT) {
                    hit_x = 1; // Hit a wall horizontally.
                    hit_y = 0;
                } else {
                    hit_x = 0;
                    hit_y = 1; // Hit a wall vertically.
                }
            }
            lastFrameTime = now;
        }
    }
}

/* ****************************** */
void display(void) {
    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity(); //clear the matrix
    /* The default GL window is [-1,1]x[-1,1] with the origin in the
     center.
     The points are in the range (0,0) to (WINSIZE,WINSIZE), so they
     need to be mapped to [-1,1]x [-1,1]x */
    glScalef(2.0/WINDOWSIZE, 2.0/WINDOWSIZE, 1.0);
    glTranslatef(-WINDOWSIZE/2, -WINDOWSIZE/2, 0);
    
    /* Draw the polygon and its visible area. */
    // Draw polygon.
    glColor3fv(red);
    drawPolygon();
    
    if (!hasFinishedPolygon) {
        drawCirclesAroundVertices();
    }
    if (insidePolygon(visiblityPoint)) {
        // Draw visible area.
        glColor3fv(green);
        drawVisibleAreaSegments();
        
        // Fill visible area.
        for (int i = 0; i < visibleAreaSegments.size(); i++) {
            segment2D s = visibleAreaSegments[i];
            fillTraingle(s.start, s.end, visiblityPoint);
        }
        // Draw visibility point as a filled circle.
        glColor3fv(yellow);
        drawVisbilityPoint();
    }
    
    /* execute the drawing commands */
    glFlush();
}

/* ****************************** */
void keypress(unsigned char key, int x, int y) {
    switch(key) {
        case 'q':
            // Exits the program.
            exit(0);
            break;
        case 'e':
            // Erases the polygon.
            erase();
            break;
        case 's':
            // Clears the visibility area.
            clearVectors();
            glutPostRedisplay();
            break;
        case 'm':
            // Starts automatic movements.
            startMoving();
            break;
    }
}

/* Handler for window re-size event. Called back when the window first appears and
 whenever the window is re-sized with its new width and height */
void reshape(GLsizei width, GLsizei height) {  // GLsizei for non-negative integer
    // Set the viewport to cover the new window
    glViewport(0, 0, width, height);
    
    glMatrixMode(GL_PROJECTION);  // To operate on the Projection matrix
    glLoadIdentity();             // Reset
    gluOrtho2D(0.0, (GLdouble) width, 0.0, (GLdouble) height);
}
