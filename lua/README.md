# Lua recipes

`recipes.json` is the shared script catalog for the website's `/code` reset
picker and `/docs/lua/recipes` code blocks. Edit a recipe's `code` here to update
both surfaces. Keep IDs stable; the docs reference them with
`<LuaRecipeCode id="…" />`.

Each entry is a complete replacement program. Wiring and usage instructions
live in `docs/lua/recipes.mdx`. When changing the default recipe, also keep the
firmware's `tools/rv1106-mcu-lua/default.lua` in mighty-core aligned.
