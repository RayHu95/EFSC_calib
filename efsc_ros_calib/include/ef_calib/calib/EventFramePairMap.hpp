#ifndef ef_calib_EventFramePairMap_HPP
#define ef_calib_EventFramePairMap_HPP

#include <shared_mutex>
#include <map>
#include <iterator>
//#include <set>
#include <memory>

#include <ef_calib/calib/EventFramePair.hpp>

namespace ef_calib {
    namespace calib {

    class EventFramePairMap {
    public:
        typedef std::shared_ptr<EventFramePairMap> Ptr;

        EventFramePairMap():subMap(nullptr){};

        // EventFramePairMutex_
        inline void PairLockShared() const {
            EventFramePairMutex_.lock_shared();
        }
        inline void PairUnLockShared() const {
            EventFramePairMutex_.unlock_shared();
        }

        //---------------------------- single Pair --------------------------------//
        inline void addPair(const CalibPair::Ptr &es_fr){
            std::unique_lock lock(EventFramePairMutex_);
            pair_vector[es_fr->time] = es_fr;
        }
        inline bool hasPair(uint64_t timestamp) const {
            std::shared_lock lock(EventFramePairMutex_);
            return pair_vector.find(timestamp) != pair_vector.end();
        }
        inline CalibPair::Ptr getPair(uint64_t timestamp) const {
            std::shared_lock lock(EventFramePairMutex_);
            auto result = pair_vector.find(timestamp);
            return result == pair_vector.end() ? nullptr : result->second;
        }
        inline bool empty() const {
            std::shared_lock lock(EventFramePairMutex_);
            return pair_vector.empty();
        }
        void removePair(uint64_t timestamp){
            // erase the pairs_(timestamp) until there is no pairs_(timestamp) in it
            EventFramePairMap *map = this;
            while (map != nullptr) {
                if (map->hasPair(timestamp)) {
                    map->EventFramePairMutex_.lock();
                    map->pair_vector.erase(timestamp);
                    map->EventFramePairMutex_.unlock();

                    map = map->subMap.get();
                } else {
                    break;
                }
            }
        }

        //----------------------------- multiple Pairs --------------------------------//
        //require shared mutex when using.
        inline const std::map<uint64_t, CalibPair::Ptr> &EventFramePairs() const {
            return pair_vector;
        }
        inline std::map<uint64_t, CalibPair::Ptr> copyPairs() const {
            std::shared_lock lock(EventFramePairMutex_);
            return pair_vector;
        }
        inline int PairNum() const {
            std::shared_lock lock(EventFramePairMutex_);
            return pair_vector.size();
        }

        //*******@brief clear map
        void clear() {
            EventFramePairMap *map = this;
            while (map != nullptr) {
                std::unique_lock lock(map->EventFramePairMutex_);
                map->pair_vector.clear();
                map = map->subMap.get();
            }
        }


        //*******@brief Erase EventSlices in RemoveList.
        inline void cleanMap() {
            EventFramePairsToBeErasedMutex_.lock();
            for (uint64_t timeStamp: EventFramePairsToBeErased_) {
                removePair(timeStamp);
            }
            EventFramePairsToBeErased_.clear();
            EventFramePairsToBeErasedMutex_.unlock();
        }
        //*******@brief Add pair to RemoveList. Remember to clear observations.
        inline void requestRemovePair(uint64_t timestamp) {
            if (auto bf = getPair(timestamp)) {
                std::scoped_lock lock(EventFramePairsToBeErasedMutex_);
                EventFramePairsToBeErased_.push_back(timestamp);
            }
        }

        EventFramePairMap::Ptr subMap; // parent have no mutex control over child, but part elements copied to child

    protected:
        std::map<uint64_t, CalibPair::Ptr> pair_vector;
        mutable std::shared_mutex EventFramePairMutex_;

        std::vector<uint64_t> EventFramePairsToBeErased_;
        mutable std::mutex EventFramePairsToBeErasedMutex_;
    };
} // calib namespace
}

#endif //ef_calib_EventFramePairMap_HPP
