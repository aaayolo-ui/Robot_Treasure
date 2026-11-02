import { eq } from "drizzle-orm";
import { getDb } from "../db";
import { monitorState } from "../db/schema";
import map from "./map.json";

const ONLINE_MS = 10000;
const COMMAND_MS = 15000;
const json = (data: unknown, status = 200) => Response.json(data, { status, headers: { "Cache-Control": "no-store" } });
async function read(key: string) {
  const row = await getDb().select().from(monitorState).where(eq(monitorState.key, key)).get();
  return row ? { ...row, data: JSON.parse(row.value) } : null;
}
async function write(key: string, data: unknown) {
  const value = JSON.stringify(data), updatedAt = Date.now();
  await getDb().insert(monitorState).values({ key, value, updatedAt }).onConflictDoUpdate({ target: monitorState.key, set: { value, updatedAt } });
}
const empty = () => ({ port: "", baud: 115200, connected: false, serial_enabled: false, error: "桥接电脑离线", last_receive_ms: 0, valid_frames: 0, range_stats: {}, vehicle_state: null, fault_reason: null, status_time_ms: 0, gyro: null, gray: { front: null, rear: null }, projection: null, channels: Object.fromEntries([1,2,3,4].map(n => [String(n), { mm: null, confidence: null, time_ms: 0, status: null }])), position: null, map_match: null, track_cursor: 0, track_generation: 0 });
export async function handle(request: Request): Promise<Response> {
  try {
    const url = new URL(request.url), path = url.pathname;
    if (request.method === "POST" && request.headers.get("origin") && request.headers.get("origin") !== url.origin) return json({error:"请求来源不匹配"},403);
    if (request.method === "POST" && !request.headers.get("content-type")?.startsWith("application/json")) return json({error:"需要JSON请求"},415);
    if (request.method === "GET" && path === "/api/map") return json(map);
    if (request.method === "POST" && path === "/api/bridge") {
      if (Number(request.headers.get("content-length")) > 262144) return json({error:"上传数据过大"},413);
      const text = await request.text();
      if (text.length > 262144) return json({error:"上传数据过大"},413);
      const data = JSON.parse(text);
      if (!data.snapshot || typeof data.snapshot.connected !== "boolean" || !Array.isArray(data.ports) || typeof data.session !== "string" || data.session.length > 80 || !Array.isArray(data.path?.points) || !Number.isSafeInteger(data.path.cursor) || data.path.cursor < 0 || !Number.isSafeInteger(data.path.generation) || data.path.generation < 0) return json({error:"上传格式无效"},400);
      data.path.points = data.path.points.slice(-800);
      data.ports = data.ports.filter((p: unknown) => typeof p === "string" && /^COM\d{1,3}$/i.test(p));
      delete data.snapshot.raw_log_path;
      await write("snapshot", data);
      const cmd = (await read("command"))?.data;
      return json({ok:true, command: cmd && cmd.session === data.session && Date.now()-cmd.issuedAt < COMMAND_MS ? cmd : null});
    }
    const saved = await read("snapshot"), online = Boolean(saved && Date.now()-saved.updatedAt < ONLINE_MS);
    const data = saved?.data;
    const cmd = (await read("command"))?.data;
    const command = cmd ? {id:cmd.id, acknowledged:data?.ack?.id === cmd.id, result:data?.ack?.id === cmd.id ? data.ack.result : null, expired:Date.now()-cmd.issuedAt >= COMMAND_MS} : null;
    if (request.method === "GET") {
      if (path === "/api/laser") {
        const snapshot = data?.snapshot || empty();
        const gyro=snapshot.gyro ? {...snapshot.gyro,data_age_ms:(snapshot.gyro.data_age_ms || 0)+Math.max(0,Date.now()-(saved?.updatedAt || 0))} : null;
        return json({...snapshot, gyro, connected:online && snapshot.connected, serial_enabled:online && snapshot.serial_enabled, bridge_online:online, bridge_received_ms:saved?.updatedAt || 0, command, projection:online ? snapshot.projection : null, error:online ? snapshot.error : "桥接电脑离线"});
      }
      if (path === "/api/ports") return json({ports:online ? data.ports : [], selected:data?.snapshot?.port || ""});
      if (path === "/api/path") {
        const after = Number(url.searchParams.get("after") || "0");
        if (!Number.isSafeInteger(after) || after < 0) return json({error:"轨迹游标无效"},400);
        const trajectory = data?.path || {cursor:0,generation:0,points:[]};
        return json({...trajectory, points:trajectory.points.filter((p: {id:number}) => p.id > after), truncated:Boolean(trajectory.points.length && after < trajectory.points[0].id-1)});
      }
    }
    if (request.method === "POST" && ["/api/serial","/api/reset-position"].includes(path)) {
      if (!online) return json({error:"桥接电脑离线，请先在电脑启动公网桥接程序"},409);
      if (command && !command.acknowledged && !command.expired) return json({error:"上一条指令尚未确认"},409);
      const body = await request.json() as {action?:string,port?:string};
      const action = path === "/api/reset-position" ? "reset-position" : body.action;
      if (!["connect","disconnect","select","reset-position"].includes(action || "")) return json({error:"操作无效"},400);
      if (["connect","select"].includes(action || "") && !data.ports.includes(body.port)) return json({error:"该电脑没有这个串口"},400);
      const id=crypto.randomUUID();
      await write("command", {id,session:data.session,issuedAt:Date.now(),action,port:body.port || data.snapshot.port});
      return json({queued:true,commandId:id,port:body.port || data.snapshot.port},202);
    }
    return json({error:"Not found"},404);
  } catch (e) {
    if (e instanceof SyntaxError) return json({error:"JSON格式无效"},400);
    console.error("Monitor request failed", e instanceof Error ? e.message : "unknown");
    return json({error:"监控服务暂不可用"},503);
  }
}
