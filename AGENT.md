# AGENT.md

## 対象
- このリポジトリは `STM32CubeIDE` プロジェクト。
- GitHub 上の同一リポジトリを、複数 PC・同一 GitHub アカウントで安全に運用する。

## 関連リポジトリ参照（PC側上位層）
- 参照先: `/home/rui3510/R2_ctrl`
- 用途: 上位層（PC側）仕様・通信仕様・タスク状況の確認用。
- 運用: このMCUリポジトリには取り込まない（コピー・サブモジュール化しない）。必要時にローカル参照のみ行う。

## 運用ルール（衝突回避）
1. `main` に直接作業しない。必ず作業ブランチを作る。  
   例: `feature/can-tx-fix`, `fix/timer-overflow`
2. 作業開始時は必ず最新化する。  
   `git checkout main && git pull --rebase origin main`
3. 作業ブランチ作成後、こまめにコミットする（1変更1コミット）。
4. Push 前に必ず `main` を取り込み、リベースしてから Push。  
   `git fetch origin && git rebase origin/main`
5. `main` への反映は Pull Request で行う（自己レビューでも可）。

## PCごとの初期設定（1回だけ）
各PCで以下を設定する。

```bash
git config --global pull.rebase true
git config --global rebase.autoStash true
git config --global fetch.prune true
```

推奨: どのPCのコミットか識別しやすくするため、`user.name` を少し分ける。

```bash
# PC-A
git config --global user.name "YourName-PC-A"
# PC-B
git config --global user.name "YourName-PC-B"
```

## 毎回の作業手順
1. 作業開始前（必須）
```bash
git checkout main
git pull --rebase origin main
git checkout -b feature/<task-name>
```

2. 実装中
```bash
git add <files>
git commit -m "feat: <what changed>"
```

3. Push 前（必須）
```bash
git fetch origin
git rebase origin/main
git push -u origin feature/<task-name>
```

4. GitHub で PR 作成 → `main` にマージ

## STM32CubeIDE プロジェクトでの注意
- 競合しやすい重要ファイル: `*.ioc`  
  同時編集しない。編集担当を1人に固定して短時間でマージする。
- 自動生成コードを再生成した場合は、生成差分をまとめて1コミットにする。
- `Debug/` `Release/` などビルド生成物は追跡しない（`.gitignore` で除外）。
- IDEワークスペース設定（プロジェクト外 `.metadata`）はコミットしない。

## PC切替時チェックリスト
- PCを離れる前: `git status` がクリーンか確認
- 未Pushコミットがないか確認: `git log --oneline origin/main..HEAD`
- 必要なら Push してから終了
- 別PCで作業開始時は必ず `git pull --rebase`

## 競合が起きたとき
1. 先に `git fetch origin` して状況確認
2. `git rebase origin/main` で解決
3. `*.ioc` 競合時は、片方を採用して CubeMX で再生成し、ビルド確認後コミット
4. 解決後に `git push --force-with-lease`（rebase 後のみ）

## 禁止事項
- `main` で直接開発・直接 Push
- 未同期のまま別PCで作業開始
- `git push --force` の乱用（`--force-with-lease` のみ許可）
