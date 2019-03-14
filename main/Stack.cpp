#include <Arduino.h>

using namespace std;

const int MAX_STACK_SIZE = 36;

template<class T>
class Stack {
  public:
    Stack () : top_element_index(-1), m_size(0) {
        // T contents[MAX_STACK_SIZE] = {T()};
    };

    Stack (T in[]) : contents(in) {};

    void push(T input) {
        if (m_size >= MAX_STACK_SIZE) {
            Serial.println("Stack FULL --");
            return;
        }

        top_element_index ++;
        contents[top_element_index] = input;
        m_size ++;
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
        m_size --;
    }

    bool empty() {
        return m_size <= 0;
    }

    int size() {
        return m_size;
    }

private:
    T contents[MAX_STACK_SIZE];

    int top_element_index;

    int m_size;

};
