-- caption-bottom.lua
-- Pandoc Lua filter: renders simple tables (no row/col spans) as a
-- non-longtable `table` + `tabular` environment with the caption placed
-- AFTER the tabular (i.e. at the bottom) instead of pandoc's default
-- longtable placement (caption at the top).
--
-- Usage:
--   pandoc input.md -o output.pdf --lua-filter=caption-bottom.lua
--
-- Limitations: assumes each cell contains a single paragraph/plain block
-- (no multi-paragraph cells, no rowspan/colspan). This matches typical
-- simple markdown pipe tables.

local function inlines_to_latex(inlines)
  local doc = pandoc.Pandoc({ pandoc.Plain(inlines or {}) })
  local tex = pandoc.write(doc, 'latex')
  -- parentheses force a single return value: string.gsub returns
  -- (string, count), and without this the extra count value leaks
  -- into callers like table.insert() as a spurious extra argument.
  return (tex:gsub('\n+$', ''))
end

local align_map = {
  AlignLeft = 'l',
  AlignRight = 'r',
  AlignCenter = 'c',
  AlignDefault = 'l',
}

local function cell_text(cell)
  local block = cell.contents and cell.contents[1]
  if block and block.content then
    return (inlines_to_latex(block.content))
  end
  return ''
end

function Table(tbl)
  local caption_inlines = {}
  if tbl.caption and tbl.caption.long and tbl.caption.long[1] then
    caption_inlines = tbl.caption.long[1].content
  end

  local colspec = ''
  for _, cs in ipairs(tbl.colspecs) do
    colspec = colspec .. (align_map[cs[1]] or 'l')
  end

  local lines = {}
  table.insert(lines, '\\begin{table}[H]')
  table.insert(lines, '\\centering')
  table.insert(lines, '\\begin{tabular}{' .. colspec .. '}')
  table.insert(lines, '\\toprule')

  if tbl.head and tbl.head.rows and #tbl.head.rows > 0 then
    for _, row in ipairs(tbl.head.rows) do
      local cells = {}
      for _, cell in ipairs(row.cells) do
        table.insert(cells, cell_text(cell))
      end
      table.insert(lines, table.concat(cells, ' & ') .. ' \\\\')
    end
    table.insert(lines, '\\midrule')
  end

  for _, body in ipairs(tbl.bodies) do
    for _, row in ipairs(body.body) do
      local cells = {}
      for _, cell in ipairs(row.cells) do
        table.insert(cells, cell_text(cell))
      end
      table.insert(lines, table.concat(cells, ' & ') .. ' \\\\')
    end
  end

  table.insert(lines, '\\bottomrule')
  table.insert(lines, '\\end{tabular}')

  if #caption_inlines > 0 then
    table.insert(lines, '\\caption{' .. inlines_to_latex(caption_inlines) .. '}')
  end

  table.insert(lines, '\\end{table}')

  return pandoc.RawBlock('latex', table.concat(lines, '\n'))
end
