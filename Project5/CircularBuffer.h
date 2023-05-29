#ifndef MY_CIRCULAR_BUFFER
#define MY_CIRCULAR_BUFFER

#include <vector>

template <typename T>
class CircularBuffer
{
public:
    explicit CircularBuffer(size_t capacity): m_buffer(capacity),m_head(0),m_full(false) {}

    void insert(const T& value) {
        m_buffer[m_head] = value;
        m_head = (m_head + 1) % m_buffer.size();
        if (m_head == 0) {
            m_full = true;  // Buffer becomes full when head wraps around
        }

    }


    // Accessors 
    T getValue(size_t index) const {
        if (m_full) {
            index = (m_head + index) % m_buffer.size();
        }
        else {
            index = (m_head - index - 1 + m_buffer.size()) % m_buffer.size();
        }
        return m_buffer[index];
    }

    size_t size() const{ return m_buffer.size(); }

    bool isFull() const { return m_full; }

    size_t getHead() const { return m_head; }

    T back() const
    { 
        if (m_head == 0)
            return m_buffer[m_buffer.size()-1];
        else
            return m_buffer[m_head - 1];
    
    }

private:
    std::vector<T> m_buffer;
    size_t m_head;
    bool m_full;
};

#endif    // MY_CIRCULAR_BUFFER