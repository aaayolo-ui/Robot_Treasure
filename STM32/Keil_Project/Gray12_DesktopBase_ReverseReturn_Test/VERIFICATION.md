# 验证记录

- 桌面原版全部 268 个文件：修改前后 SHA-256 完全一致，文件集合也一致。
- Keil 项目 65 个 FilePath：无重复，全部存在且位于本新工程内；Include Paths 存在且无重复。
- Core、Hardware、Drivers 与桌面原版逐文件内容一致。
- 循迹 PID、平顺控制、轮速 PI、底盘映射与桌面原版内容一致；全部 ROUTE 参数一致。
- 任务循环 10 处领头灰度取样均紧接全黑返程停车判断；换向后缓存立即失效。
- ARMCC 5.06 全量编译：0 Error(s), 0 Warning(s)。
- 仅编译通过，尚未完成硬件验证。未执行硬件运行/延迟测量/倒车转弯验证。

以上为文件、源码和工程结构检查，不代表运行时测试。

## Second-junction return update

- Outbound junction 1 turns normally; junction 2 brakes before reverse.
- The completed outbound turn is recorded and inverted for the return junction; right-priority confirmation cannot override it.
- New runs reset the recorded turn; reverse turn completion consumes it.
- Desktop source hashes unchanged (268 files); 65 project references exist without duplicates.
- ARMCC5 rebuild: 0 Error(s), 0 Warning(s). Hardware route and timing remain untested.

## Fifth-junction return (supersedes second-junction update)

- Junctions 1-4 complete normally; junction 5 brakes and starts reverse.
- Four completed outbound turns are pushed with a capacity guard, then replayed in reverse order with opposite directions. Entries pop only after return turns complete; new runs reset the count.
- Desktop source: all 268 file hashes unchanged.
- ARMCC5 full rebuild: 0 Error(s), 0 Warning(s). No hardware test performed.

## Premature DONE fix

- Finish requires no remaining return turns, rearmed valid-line follow (or its recovery), and rear all-black. Intermediate junctions and turning states cannot finish.
- Evaluated the source finish predicate against 5760 combinations of direction, remaining turns, arming/latch/valid flags, route/resume state, and black detection; all matched the expected gate. This is a predicate check, not a full state-machine or hardware test.
- Desktop: 268 file hashes unchanged. ARMCC5 full rebuild: 0 Error(s), 0 Warning(s).
- Hardware return-to-start validation still required.
