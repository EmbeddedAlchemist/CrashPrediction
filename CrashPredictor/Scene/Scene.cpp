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

//将标量和角度正交分解成向量
static b2Vec2 constructVecFromScalerAndAngle(float scaler, float angle) {
    return b2Vec2(
        scaler * std::cos(angle),
        scaler * std::sin(angle)
    );
}

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
    //res.ApplyForce(src.GetForce())
    if (*reinterpret_cast<int *>(res.GetUserData().pointer) == 1) {//判断是不是车辆
        // 由于box2D没有提供方法来获取施加到物体上的力，现在根据车辆信息重新设置
        Vehicle *veh = reinterpret_cast<Vehicle *>(res.GetUserData().pointer);
        auto acc = veh->getAccleration();
        auto position = veh->getPosition();
        res.ApplyForceToCenter(constructVecFromScalerAndAngle(acc.linear * res.GetMass(), position.angle), true);
        res.ApplyTorque(acc.angular * res.GetInertia(), true);
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
    def.linearVelocity = constructVecFromScalerAndAngle(report.velocity.linear, report.position.angle);
    def.angularVelocity = report.velocity.angular;

    //def.userData.pointer = reinterpret_cast<std::uintptr_t>(new std::weak_ptr<Vehicle>(vehicle));
    def.userData.pointer = reinterpret_cast<std::uintptr_t>(&*vehicle);
}

void Scene::createB2FixtureDefByVehicle(const std::shared_ptr<Vehicle> &vehicle, b2FixtureDef &def, b2PolygonShape &shape) {
    shape.SetAsBox(vehicle->info.size.length / 2, vehicle->info.size.width / 2);
    def.shape = &shape;
    def.density = vehicle->info.weight / (vehicle->info.size.width * vehicle->info.size.length);
    def.friction = 0.f;
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

void Scene::run(long long ms) {
    std::unique_lock<std::mutex> lock(worldEmulateLock);
    while (ms > 10) {
        world->Step(0.01, 6, 4);
        ms -= 10;
    }
    while (ms > 1) {
        world->Step(0.001, 2, 1);
        ms -= 1;
    }
}

std::vector<CrashInfo> Scene::predict(const PredictOption &option)
{
    std::unique_lock<std::mutex> lock(worldEmulateLock);
    ContactCallback contactListener(option);
    world->SetContactListener(&contactListener);
    for (float i = 0; i <= option.emulateTime; i += option.stepTime) {
        world->Step(option.stepTime, 6, 4);
    }
    world->SetContactListener(nullptr);
    return std::vector<CrashInfo>();
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
        Vehicle *vehicle = reinterpret_cast<Vehicle *>(veh->GetUserData().pointer);

        //LOG_D << "Trying to calc recommend speed for vehicle " << vehicle->getId();

        b2Body *matchedTrafficLight = [&]() {
            b2Body *mostMatch = nullptr;
            //尝试寻找和车头方向相近的交通灯
            for (auto tl : trafficLightList) {
                float vehTriAngleDiff = veh->GetAngle() - tl->GetAngle();
                while (vehTriAngleDiff > b2_pi)
                    vehTriAngleDiff -= 2 * b2_pi;
                while (vehTriAngleDiff < -b2_pi)
                    vehTriAngleDiff += 2 * b2_pi;
                if ((mostMatch == nullptr && std::abs(vehTriAngleDiff) < 0.39269908169872415480783042290994) ||
                    (mostMatch != nullptr && std::abs(vehTriAngleDiff) < std::abs(veh->GetAngle() - mostMatch->GetAngle())))
                    mostMatch = tl;
            }
            return mostMatch;

            }();
            if (matchedTrafficLight == nullptr) {
                //找不到匹配的交通灯
                //LOG_V << "No matched traffic light.";
                continue;
            }

            TrafficLight *trafficLight = reinterpret_cast<TrafficLight *>(matchedTrafficLight->GetUserData().pointer);
            
            float leaseRemainTime = 0, mostRemainTime = 0;
            float lestRecommendSpeed = 0, mostRecommendSpeed = 0;

            if (trafficLight->allowTravel == true) {
                leaseRemainTime = mostRemainTime = trafficLight->getTimeLeft();
            }
            else {
                leaseRemainTime = trafficLight->getTimeLeft();
                mostRemainTime = leaseRemainTime + 30; // TODO:增加交通灯绿灯时间回报
            }

            float distance = b2Distance(veh->GetPosition(), matchedTrafficLight->GetPosition());
            if (distance > 1000.f) {
                //LOG_V << "Too far";
                continue;
            }

            lestRecommendSpeed = distance / mostRemainTime;
            mostRecommendSpeed = distance / leaseRemainTime;

            float recommendVelocity = (lestRecommendSpeed + mostRecommendSpeed) / 2;

            if (trafficLight->allowTravel && recommendVelocity < 30.0 / 3.6)
                recommendVelocity = 30.0 / 3.6;
            if (recommendVelocity < 5.55 || recommendVelocity>17.0f) {
                //LOG_V << "Too slow";
                continue;
            } //速度太慢不做推荐，乖乖等红灯
            
            //LOG_D << "Recommend: " << recommendVelocity << "m/s";

            vehicle->getPredictStatusBufRef().recommendSpeed.emplace(recommendVelocity);
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

void Scene::vehicleReport(std::shared_ptr<Vehicle> &vehicle) {
    std::lock_guard<std::mutex> worldLockGrand(worldEmulateLock);
    b2Body *body = vehicle->getB2Body();

    //box2D引擎没有提供有效方式来清除附加在某一物体上的力，现在通过休眠物体来间接清除物体上的力
    //这将归零物体的休眠时间、线速度、角速度、力和扭矩，休眠时间无关紧要，其余的值接下来会重新计算。
    auto isAwake = body->IsAwake();
    body->SetAwake(false);
    body->SetAwake(true);
    auto report = vehicle->getLastReport();
    auto position = report.position;
    body->SetTransform(b2Vec2(position.x, position.y), position.angle);
    auto velocity = report.velocity;

    body->SetLinearVelocity(constructVecFromScalerAndAngle(velocity.linear, position.angle));
    body->SetAngularVelocity(velocity.angular);
    auto accleration = vehicle->getAccleration();
    body->ApplyForceToCenter(constructVecFromScalerAndAngle(accleration.linear * body->GetMass(), position.angle), true);
    body->ApplyTorque(accleration.angular * body->GetInertia(), true);

}

void Scene::trafficLightReport(std::shared_ptr<TrafficLight> &trafficLight, const TrafficLightReport &report) {
    if (!trafficLight->isBindWithB2Body()) {
        trafficLight->bindB2Body(createB2BodyByTrafficLight(trafficLight));
        LOG_D << "Traffic light " << trafficLight->getId() << " bind with b2Body " << trafficLight->getB2Body();
    }
}

Scene::Scene()
    :world(std::make_unique<b2World>(b2Vec2_zero)) {
    world->SetAutoClearForces(false);
    LOG_D << "world inited";
}

