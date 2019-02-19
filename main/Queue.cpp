using namespace std;

const int MAX_QUEUE_SIZE = 100;

template<class T>
class Queue {
  public:
    Queue () :m_front_index(0), m_rear_index(-1), m_size(0) {};

    Queue (T in[]) : contents(in) {};

    void push(T input) {
        if (m_size >= MAX_QUEUE_SIZE) {
            return;
        }

        m_rear_index ++;
        contents[m_rear_index] = input;
        m_size ++;
    }

    T front() {
        if (empty())
            return T();
        else
            return contents[m_front_index % MAX_QUEUE_SIZE];
    }

    void pop() {
        if (empty())
            return;

        contents[m_front_index % MAX_QUEUE_SIZE] = {};
        m_front_index ++;
        m_size --;
        return;
    }

    int size() {
        return m_size;
    }

    bool empty() {
        return m_size <= 0;
    }

private:
    T contents[MAX_QUEUE_SIZE];

    int m_front_index;

    int m_rear_index;

    int m_size;

};
