#!/usr/bin/env bash
# Git Aliases and Shortcuts Setup Script
# Usage: source .git-aliases.sh or add to your ~/.bashrc / ~/.config/fish/config.fish

# === Quick Status & Info ===
alias gs='git status -sb'
alias gss='git status'
alias gl='git log --oneline --graph --decorate -20'
alias gla='git log --oneline --graph --decorate --all -20'
alias glf='git log --oneline --graph --decorate --all'  # Full history

# === Branch Operations ===
alias gb='git branch -vv'
alias gba='git branch -a'
alias gco='git checkout'
alias gcb='git checkout -b'
alias gbd='git branch -d'
alias gbD='git branch -D'

# === Staging & Commits ===
alias ga='git add'
alias gaa='git add -A'
alias gap='git add -p'  # Interactive staging
alias gc='git commit -v'
alias gcm='git commit -m'
alias gca='git commit --amend'
alias gcan='git commit --amend --no-edit'

# === Diff ===
alias gd='git diff'
alias gdc='git diff --cached'
alias gds='git diff --staged'

# === Pull & Push ===
alias gp='git pull'
alias gpr='git pull --rebase'
alias gpu='git push'
alias gpuf='git push --force-with-lease'  # Safer force push
alias gpsu='git push --set-upstream origin $(git branch --show-current)'

# === Fetch & Remote ===
alias gf='git fetch'
alias gfa='git fetch --all'
alias gr='git remote -v'

# === Stash ===
alias gst='git stash'
alias gsta='git stash apply'
alias gstl='git stash list'
alias gstp='git stash pop'
alias gstd='git stash drop'

# === Merge & Rebase ===
alias gm='git merge'
alias grb='git rebase'
alias grbi='git rebase -i'
alias grbc='git rebase --continue'
alias grba='git rebase --abort'

# === Reset & Clean ===
alias grh='git reset HEAD'
alias grhh='git reset --hard HEAD'
alias gclean='git clean -fd'

# === Utility Functions ===

# Show changed files
gch() {
    git diff --name-status "$@"
}

# Undo last commit (keep changes)
gundo() {
    git reset --soft HEAD~1
}

# Quick commit with message
gcq() {
    git add -A && git commit -m "$*"
}

# Create and checkout new branch from main/master
gcnb() {
    local main_branch=$(git symbolic-ref refs/remotes/origin/HEAD | sed 's@^refs/remotes/origin/@@')
    git checkout $main_branch && git pull && git checkout -b "$1"
}

# Delete merged branches (excluding main/master/develop)
gbclean() {
    git branch --merged | grep -v "\*\|main\|master\|develop" | xargs -n 1 git branch -d
}

# Show files in last commit
glast() {
    git show --name-status HEAD
}

# Interactive log with search
glg() {
    git log --oneline --graph --decorate --all | head -n 50
}

# Show git statistics
gstats() {
    echo "=== Repository Statistics ==="
    echo "Total commits: $(git rev-list --count HEAD)"
    echo "Contributors: $(git shortlog -sn | wc -l)"
    echo "Branches: $(git branch -a | wc -l)"
    echo "\n=== Top Contributors ==="
    git shortlog -sn --no-merges | head -10
}

echo "Git aliases loaded! Type 'alias | grep git' to see all aliases."
