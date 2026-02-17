#!/bin/bash

# IIT 폴더의 루트로 이동 (스크립트 위치와 상관없이)
REPO_ROOT="/home/chanyoungko/IIT"
cd "$REPO_ROOT" || exit

echo "--- Git Sync Start ---"

# 1. 모든 변경사항 스테이징
git add .

# 2. 커밋 메시지 생성 (현재 시간 포함)
COMMIT_MSG="Auto sync: $(date '+%Y-%m-%d %H:%M:%S')"
if git commit -m "$COMMIT_MSG"; then
    echo "✅ Changes committed: $COMMIT_MSG"
else
    echo "⚠️ No changes to commit or commit failed."
fi

# 3. GitHub로 푸시
echo "🚀 Pushing to GitHub (main branch)..."
if git push origin main; then
    echo "✅ Successfully pushed to GitHub."
else
    echo "❌ Push failed. Please check your internet connection or credentials."
fi

echo "--- Git Sync End ---"
