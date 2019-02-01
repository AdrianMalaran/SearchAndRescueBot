using namespace std;

const int MAX_STACK_SIZE = 100;

template<class T>
class Stack {
  public:
    Stack () : top_element_index(-1), size(0) {
        // T contents[MAX_STACK_SIZE] = {T()};
    };

    Stack (T in[]) : contents(in) {};

    void push(T input) {
        if (size >= MAX_STACK_SIZE) {
            // printf("No more room");
            return;
        }

        top_element_index ++;
        contents[top_element_index] = input;
        size ++;
    }

    T top() {
        if (empty())
            return T();
        return contents[top_element_index];
    }

    void pop() {
        if (empty())
            return;

        contents[top_element_index] = {};
        top_element_index --;
        size --;
    }

    bool empty() {
        return size <= 0;
    }

private:
    T contents[MAX_STACK_SIZE];

    int top_element_index;

    int size;


};
