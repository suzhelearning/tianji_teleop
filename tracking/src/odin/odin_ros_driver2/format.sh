#!/bin/bash

# 检查参数
if [ $# -ne 1 ]; then
    echo "用法: $0 <目录或文件>"
    exit 1
fi

TARGET="$1"

# 检查 clang-format 是否存在
if ! command -v clang-format &> /dev/null; then
    echo "❌ 错误：clang-format 未安装"
    exit 1
fi

# 使用 realpath 规范化路径
TARGET_ABS=$(realpath "$TARGET" 2>/dev/null)
if [ -z "$TARGET_ABS" ]; then
    echo "❌ 错误：'$TARGET' 路径无效"
    exit 1
fi

FILES=()

if [ -f "$TARGET_ABS" ]; then
    # 是文件
    case "${TARGET_ABS,,}" in
        *.h|*.cpp|*.c)
            FILES+=("$TARGET_ABS")
            ;;
        *)
            echo "⚠️  跳过：'$TARGET_ABS' 不是 .h/.cpp/.c 文件"
            exit 0
            ;;
    esac
elif [ -d "$TARGET_ABS" ]; then
    # 是目录：递归查找所有 .h .cpp .c 文件
    while IFS= read -r -d '' file; do
        FILES+=("$file")
    done < <(find "$TARGET_ABS" -type f \( -iname "*.h" -o -iname "*.hpp" -o -iname "*.cpp" -o -iname "*.c" \) -print0 2>/dev/null)
else
    echo "❌ 错误：'$TARGET_ABS' 既不是文件也不是目录"
    exit 1
fi

# 检查是否有文件需要格式化
if [ ${#FILES[@]} -eq 0 ]; then
    echo "⚠️  未找到 .h/.cpp/.c 文件"
    exit 0
fi

# 格式化文件
echo "📝 找到 ${#FILES[@]} 个文件，开始格式化..."
for file in "${FILES[@]}"; do
    echo "  格式化: $file"
    clang-format -i "$file"
done

echo "✅ 格式化完成"