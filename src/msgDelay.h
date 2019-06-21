//
//  msgDelay.h
//  ar-facepalm
//
//  Created by Charles Holbrow on 12/9/18.
//

#ifndef msgDelay_h
#define msgDelay_h

#include <list>

template <class T>
struct MessageWithTime {
    double timeReceived;
    T message;
};

template <class T>
class MsgDelay {
private:
    std::list<MessageWithTime<T>> storage;
public:
    // when adding, timeReceived must be >= the previous add
    void add(double timeReceived, T msg) {
        if (storage.size() > 0 && storage.back().timeReceived > timeReceived) {
            // storage size > 0 and the previous message was received after this one.
            return; // fail silently
        }
        MessageWithTime<T> mwt;
        mwt.timeReceived = timeReceived;
        mwt.message = msg;
        storage.push_back(mwt);
    }
    bool hasMessageAt(double time) {
        if (storage.size() <= 0) return false;
        if (storage.front().timeReceived > time) return false;
        return true;
    }
    // Get the most recent message (up until `time`). Removes all messages
    // including the one returned from the queue. Should only be called if
    // `hasMessageAt(time)` returns true.
    T getMessageAt(double time) {
        MessageWithTime<T> mwt;
        while (storage.size() && storage.front().timeReceived <= time) {
            mwt = storage.front();
            storage.pop_front();
        }
        return mwt.message;
    };
};

#endif /* msgDelay_h */
