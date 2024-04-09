#pragma once

#include <box2d/b2_draw.h>
#include <ArduinoJson/ArduinoJson.hpp>
#include <queue>
#include <string>
#include <httplib/httplib.h>
#include <mutex>
#include <condition_variable>
#include <box2d/b2_world.h>

class DebugDraw:public b2Draw {
private:

	std::mutex mutex;

	ArduinoJson::JsonDocument jdoc;
	ArduinoJson::JsonArray jarr;

	httplib::Server svr;
	b2World *world = nullptr;

	std::unique_ptr<std::thread> svrThread;

public:
	void serverThreadTask(const std::string host, std::uint16_t port);
	virtual void DrawPolygon(const b2Vec2 *vertices, int32 vertexCount, const b2Color &color);
	virtual void DrawSolidPolygon(const b2Vec2 *vertices, int32 vertexCount, const b2Color &color);
	virtual void DrawCircle(const b2Vec2 &center, float radius, const b2Color &color);
	virtual void DrawSolidCircle(const b2Vec2 &center, float radius, const b2Vec2 &axis, const b2Color &color);
	virtual void DrawSegment(const b2Vec2 &p1, const b2Vec2 &p2, const b2Color &color);
	virtual void DrawTransform(const b2Transform &xf);
	virtual void DrawPoint(const b2Vec2 &p, float size, const b2Color &color);


	void bindToWorld(b2World *world);

	DebugDraw(const std::string host, std::uint16_t port);
	virtual ~DebugDraw();
};

