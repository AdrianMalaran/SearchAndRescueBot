// Node
class Node {
  int val;
  Node *next;
  Node() : next(nullptr) {}
  Node(int data) : val(data), next(nullptr) {}
};

class Perception {
  static void updateMap();
};

class Motion {
  public:
    static void moveForward();
    static void moveBackward();
    static void turnLeft90Degrees();
    static void turnRight90Degrees();
    static void uTurn();
};

