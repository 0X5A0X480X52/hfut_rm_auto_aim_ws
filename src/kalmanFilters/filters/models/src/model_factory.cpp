#include "models/model_factory.h"
#include <stdexcept>
#include <sstream>

ModelFactoryRegistry& ModelFactoryRegistry::getInstance() {
    static ModelFactoryRegistry instance;
    return instance;
}

void ModelFactoryRegistry::registerFactory(const std::string& name, 
                                           std::shared_ptr<ModelFactoryBase> factory) {
    if (!factory) {
        throw std::invalid_argument("Factory pointer cannot be null");
    }
    
    if (factories_.find(name) != factories_.end()) {
        // 如果已存在同名工厂，发出警告但允许覆盖
        // 在实际应用中可以根据需要选择抛出异常
    }
    
    factories_[name] = factory;
}

std::unique_ptr<Models> ModelFactoryRegistry::createModel(const std::string& name, 
                                                          const ModelConfig& config) {
    auto it = factories_.find(name);
    if (it == factories_.end()) {
        std::stringstream ss;
        ss << "Factory '" << name << "' not found. Available factories: ";
        for (const auto& pair : factories_) {
            ss << pair.first << " ";
        }
        throw std::runtime_error(ss.str());
    }
    
    return it->second->createModel(config);
}

bool ModelFactoryRegistry::hasFactory(const std::string& name) const {
    return factories_.find(name) != factories_.end();
}

std::vector<std::string> ModelFactoryRegistry::getRegisteredFactories() const {
    std::vector<std::string> names;
    names.reserve(factories_.size());
    for (const auto& pair : factories_) {
        names.push_back(pair.first);
    }
    return names;
}
