import { integer, sqliteTable, text } from "drizzle-orm/sqlite-core";
export const monitorState = sqliteTable("monitor_state", {
  key: text("key").primaryKey(),
  value: text("value").notNull(),
  updatedAt: integer("updated_at").notNull(),
});
