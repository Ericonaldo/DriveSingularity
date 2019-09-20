#ifndef DRIVE_SINGULARITY_ACTION_H
#define DRIVE_SINGULARITY_ACTION_H

namespace ds {
namespace control {

    enum ACTION_SPACE_TYPE {DISCRETE=0, CONTINUOUS, BUCKET};

    namespace discrete {
        enum Action {
            Idle = 0,
            Faster,
            Slower,
            TurnLeft,
            TurnRight,
            Count
        };
    }

    namespace continuous {
        // TODO(ming): specify continuous action space
    }

    namespace bucket {
        // TODO(ming): specify bucket continuous action space
    }
} // namespace control
} // namespace ds


#endif //DRIVE_SINGULARITY_ACTION_H
