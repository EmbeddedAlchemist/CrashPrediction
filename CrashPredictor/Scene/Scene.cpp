#include "CrashInfo/CrashInfo.hpp"
#include "PredictOption/PredictOption.hpp"
#include "Scene.hpp"
#include <box2d/box2d.h>
#include <CrashPredictor/Vehicle/Vehicle.hpp>
#include <memory>
#include <NotNull/NotNull.hpp>
#include <vector>
#include <ExLog/ExLog.hpp>
#include <box2d/b2_math.h>
#include <cmath>
#include "ContractCallBack/ContactCallback.h"


static b2BodyDef b2GetBodyDef(b2Body &body) {
    NotNull(body);
    b2BodyDef res;
    res.allowSleep = body.IsSleepingAllowed();
    res.angle = body.GetAngle();
    res.angularDamping = body.GetAngularDamping();
    res.angularVelocity = body.GetAngularVelocity();
    res.awake = body.IsAwake();
    res.bullet = body.IsBullet();
    res.enabled = body.IsEnabled();
    res.fixedRotation = body.IsFixedRotation();
    res.gravityScale = body.GetGravityScale();
    res.linearDamping = body.GetLinearDamping();
    res.linearVelocity = body.GetLinearVelocity();
    res.position = body.GetPosition();
    res.type = body.GetType();
    res.userData = body.GetUserData();
    return res;
}

static b2FixtureDef b2GetFixtureDef(b2Fixture &fixture) {
    NotNull(fixture);
    b2FixtureDef res;
    res.density = fixture.GetDensity();
    res.filter = fixture.GetFilterData();
    res.friction = fixture.GetFriction();
    res.isSensor = fixture.IsSensor();
    res.restitution = fixture.GetRestitution();
    res.restitutionThreshold = fixture.GetRestitutionThreshold();
    res.shape = fixture.GetShape();
    res.userData = fixture.GetUserData();
    return res;
}


static b2Body &b2CopyBody(b2World &world, b2Body &src) {
    NotNull(world);
    NotNull(src);
    b2BodyDef bodydef = b2GetBodyDef(src);
    b2Body &res = *world.CreateBody(&bodydef);
    for (b2Fixture *fixture = src.GetFixtureList();
        fixture != nullptr;
        fixture = fixture->GetNext()) {
        b2FixtureDef fixturedef = b2GetFixtureDef(*fixture);
        res.CreateFixture(&fixturedef);
    }
    return res;
}

static std::unique_ptr<b2World> b2CopyWorld(b2World &src) {
    NotNull(src);
    std::unique_ptr<b2World> res = std::make_unique<b2World>(src.GetGravity());
    res->SetAllowSleeping(src.GetAllowSleeping());
    res->SetAutoClearForces(src.GetAutoClearForces());
    res->SetContinuousPhysics(src.GetContinuousPhysics());
    res->SetSubStepping(src.GetSubStepping());
    res->SetWarmStarting(src.GetWarmStarting());
    for (b2Body *body = src.GetBodyList();
        body != nullptr;
        body = body->GetNext()) {
        b2CopyBody(*res, *body);
    }
    return res;
}

void Scene::createB2BodyDefByVehicle(const std::shared_ptr<Vehicle> &vehicle, b2BodyDef &def)
{
    def.type = b2_dynamicBody;
    VehicleReport report = vehicle->getLastReport();
    def.position = b2Vec2(report.position.x, report.position.y);
    def.angle = report.position.angle;
    def.linearVelocity = b2Vec2(report.velocity.linear * std::cosf(report.position.angle), report.velocity.linear * std::sinf(report.position.angle));
    def.angularVelocity = report.velocity.angular;

    //def.userData.pointer = reinterpret_cast<std::uintptr_t>(new std::weak_ptr<Vehicle>(vehicle));
    def.userData.pointer = reinterpret_cast<std::uintptr_t>(&*vehicle);
}

void Scene::createB2FixtureDefByVehicle(const std::shared_ptr<Vehicle> &vehicle, b2FixtureDef &def, b2PolygonShape &shape) {
    shape.SetAsBox(vehicle->info.size.length / 2, vehicle->info.size.width / 2);
    def.shape = &shape;
    def.density = vehicle->info.weight / (vehicle->info.size.width * vehicle->info.size.length);
    def.friction = 0.5;
    //def.filter.categoryBits = 0x0001;
    //def.filter.maskBits = 0x0001;

}



b2Body *Scene::createB2BodyByVehicle(const std::shared_ptr<Vehicle> &vehicle) {
    b2BodyDef bodyDef;
    createB2BodyDefByVehicle(vehicle, bodyDef);
    b2Body *body = world->CreateBody(&bodyDef);

    b2FixtureDef fixtureDef;
    b2PolygonShape shape;
    createB2FixtureDefByVehicle(vehicle, fixtureDef, shape);
    body->CreateFixture(&fixtureDef);

    return body;
}

void Scene::createB2BodyDefByTrafficLight(const std::shared_ptr<TrafficLight> &trafficLight, b2BodyDef &def) {
    def.type = b2_staticBody;
    def.position = b2Vec2(trafficLight->stopLine.position.x, trafficLight->stopLine.position.y);
    def.angle = trafficLight->stopLine.position.angle;
    def.userData.pointer = reinterpret_cast<std::uintptr_t>(&*trafficLight);
}

void Scene::createB2FixtureDefByTrafficLight(const std::shared_ptr<TrafficLight> &trafficLight, b2FixtureDef &def, b2PolygonShape &shape) {
    shape.SetAsBox(0.1f / 2, trafficLight->stopLine.width / 2);
    def.shape = &shape;
    //def.filter.categoryBits = 0x0002;
    //def.filter.maskBits = 0x0000;
    def.isSensor = true;
}

b2Body *Scene::createB2BodyByTrafficLight(const std::shared_ptr<TrafficLight> &trafficLight)
{
    b2BodyDef bodyDef;
    b2FixtureDef fixtureDef;
    b2PolygonShape shape;
    createB2BodyDefByTrafficLight(trafficLight, bodyDef);
    createB2FixtureDefByTrafficLight(trafficLight, fixtureDef, shape);
    b2Body *body = world->CreateBody(&bodyDef);
    body->CreateFixture(&fixtureDef);
    return body;
}

Scene::Scene(Scene &scene) {
    std::lock_guard<std::mutex> worldLockGrand(scene.worldEmulateLock);
    world = b2CopyWorld(*scene.world);
}

Scene::~Scene() {

}

void Scene::run() {

}

std::vector<CrashInfo> Scene::predict(const PredictOption &option)
{
    std::unique_lock<std::mutex> lock(worldEmulateLock);
    ContactCallback contactListener(option);
    world->SetContactListener(&contactListener);
    //lock.lock();
    for (float i = 0; i <= option.emulateTime; i += option.stepTime) {
        world->Step(option.stepTime, 6, 4);
    }
    //lock.unlock();
    world->SetContactListener(nullptr);
    return contactListener.getContact();
}

void Scene::calculateRecommendVelocity() {
    std::vector<b2Body *> trafficLightList;
    std::vector<b2Body *> vehicleList;
    b2Body *bodyList = world->GetBodyList();
    while (bodyList) {
        if (*reinterpret_cast<int *>(bodyList->GetUserData().pointer) == 1)
            vehicleList.push_back(bodyList);
        if (*reinterpret_cast<int *>(bodyList->GetUserData().pointer) == 2)
            trafficLightList.push_back(bodyList);
        bodyList = bodyList->GetNext();
    }
    for (auto veh : vehicleList) {
        b2Body *matchedTrafficLight = [&]() -> b2Body *{
            b2Body *mostMatch = nullptr;
            for (auto tl : trafficLightList) {
                if (std::abs(veh->GetAngle() - tl->GetAngle()) < 0.39269908169872415480783042290994 &&
                    mostMatch == nullptr || std::abs(veh->GetAngle() - tl->GetAngle()) < std::abs(veh->GetAngle() - mostMatch->GetAngle()))
                    mostMatch = tl;
            }
            return mostMatch;
            }();
        if (matchedTrafficLight == nullptr) continue; //找不到匹配的交通灯
        
        TrafficLight *trafficLight = reinterpret_cast<TrafficLight *>(matchedTrafficLight->GetUserData().pointer);
        if (trafficLight->allowTravel == true) continue; //目前是绿灯，不需要推荐速度

        Vehicle *vehicle = reinterpret_cast<Vehicle *>(veh->GetUserData().pointer);
        float distance = (veh->GetPosition() - matchedTrafficLight->GetPosition()).Length();
        if (distance > 1000.f) continue;

        float recommendVelocity = distance / trafficLight->getTimeLeft();
        if (recommendVelocity < 5.55) continue; //速度太慢不做推荐，乖乖等红灯

        vehicle->predictStatus.recommendSpeed = recommendVelocity;
    }
}

b2World *Scene::_debugGetWorld()
{
    return &*world;
}

void Scene::vehicleRegister(std::shared_ptr<Vehicle> &vehicle) {
    b2Body *body;
    {
        std::lock_guard<std::mutex> worldLockGrand(worldEmulateLock);
        body = createB2BodyByVehicle(vehicle);
    }
    vehicle->bindB2Body(body);
    LOG_D << "Vehicle " << vehicle->getId() << " has bind with b2body " << vehicle->getB2Body();
}
void Scene::vehicleUnregister(std::shared_ptr<Vehicle> &vehicle) {
    std::lock_guard<std::mutex> worldLockGrand(worldEmulateLock);
    world->DestroyBody(vehicle->getB2Body());
}

void Scene::vehicleReport(std::shared_ptr<Vehicle> &vehicle, const VehicleReport &report) {
    std::lock_guard<std::mutex> worldLockGrand(worldEmulateLock);
    b2Body *body = vehicle->getB2Body();
    body->SetTransform(b2Vec2(report.position.x, report.position.y), report.position.angle);
    body->SetLinearVelocity(b2Vec2(report.velocity.linear * std::cos(report.position.angle), report.velocity.linear * std::sin(report.position.angle)));
    body->SetAngularVelocity(report.velocity.angular);
}

void Scene::trafficLightReport(std::shared_ptr<TrafficLight> &trafficLight, const TrafficLightReport &report) {
    if (!trafficLight->isBindWithB2Body()) {
        trafficLight->bindB2Body(createB2BodyByTrafficLight(trafficLight));
        LOG_I << "Traffic light " << trafficLight->getId() << " bind with b2Body " << trafficLight->getB2Body();
    }
}

Scene::Scene()
    :world(std::make_unique<b2World>(b2Vec2_zero)) {
    LOG_D << "world inited";
}

