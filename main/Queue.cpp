using namespace std;

const int MAX_QUEUE_SIZE = 100;

template<class T>
class Stack {
  public:
    Stack () :front_index(0), rear_index(-1), size(0) {};

    Stack (T in[]) : contents(in) {};

    void push(T input) {
        if (size >= MAX_QUEUE_SIZE) {
            return;
        }

        rear_index ++;
        contents[rear_index] = input;
        size ++;
    }

    T front() {
        if (empty())
            return T();
        else
            return contents[front_index % MAX_QUEUE_SIZE];
    }

    void pop() {
        if (empty())
            return;

        contents[front_index % MAX_QUEUE_SIZE] = {};
        front_index ++;
        return;
    }

    bool empty() {
        return size <= 0;
    }

private:
    T contents[MAX_QUEUE_SIZE];

    int front_index;

    int rear_index;

    int size;

};
