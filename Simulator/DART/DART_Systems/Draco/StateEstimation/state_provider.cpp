#include "state_provider.hpp"

StateProvider* StateProvider::getStateProvider() {
    static StateProvider stateProvider;
    return &stateProvider;
}

StateProvider::StateProvider() {}

StateProvider::~StateProvider() {}
