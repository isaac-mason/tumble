#pragma once

#include "app/app.h"

namespace examples {

/** an example adds itself from its own file: `static const examples::Registration REGISTERED(EXAMPLE_<NAME>);` */
struct Registration {
    explicit Registration(const Example &example);
};

/** the registered examples, in crashcat's order */
int count();
const Example &at(int index);

}
