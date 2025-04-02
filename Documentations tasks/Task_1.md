# Steps Taken for GitHub Task

## Task 1: Setting Up and Pushing a Folder to GitHub

### Create a Folder on the Local System:
```bash
mkdir GitHub_task
cd GitHub_task
```
This creates a directory named `GitHub_task` and navigates into it.

### Create a Program File (Example: C++ or Python) Inside the Folder:
```bash
echo '#include <iostream>\nint main() { std::cout << "Hello World!"; return 0; }' > hello.cpp
```
This creates a simple C++ program that prints "Hello World!".

### Initialize the Git Repository:
```bash
git init
```
This initializes a new Git repository inside the `GitHub_task` folder.

### Add the Remote Repository:
```bash
git remote add origin <GitHub_Repo_URL>
```
This links the local repository to the remote GitHub repository.

### Add Files to the Staging Area:
```bash
git add .
```
This stages all files for commit.

### Commit the Changes:
```bash
git commit -m "Initial commit - Added Hello World program"
```
This saves the changes in the local repository with a commit message.

### Rename the Default Branch to Master:
```bash
git branch -M master
```
This ensures that the main branch is named `master`.

### Push the Changes to GitHub:
```bash
git push -u origin master
```
This pushes the `master` branch to GitHub.

## Task 2: Merging `main` into `master`

### Check Existing Branches:
```bash
git branch -a
```
Lists all local and remote branches.

### Fetch the Latest Changes from Remote:
```bash
git fetch origin
```
Ensures local branches are up to date with the remote repository.

### Create a Local `main` Branch (If Not Present):
```bash
git checkout -b main origin/main
```
Creates a local copy of the `main` branch from GitHub.

### Merge `main` into `master`:
```bash
git checkout master
git merge main --allow-unrelated-histories
```
Since the two branches had different commit histories, this allows merging unrelated histories.

### Resolve Merge Conflicts (if any):
If conflicts occur, manually edit the conflicting files and then:
```bash
git add .
git commit -m "Resolved merge conflicts"
```

### Push the Updated `master` Branch:
```bash
git push origin master
```
Ensures the `master` branch is updated on GitHub.

### Delete the `main` Branch (Optional):
```bash
git branch -d main  # Delete locally
git push origin --delete main  # Delete on GitHub
```
Removes `main` after merging into `master`.

## Task 3: Debugging Git Merge Issues

### Error: "merge: main - not something we can merge"
Solution: Ensure `main` exists remotely and fetch the latest updates:
```bash
git fetch origin
```

### Error: "fatal: refusing to merge unrelated histories"
Solution: Allow unrelated histories:
```bash
git merge origin/main --allow-unrelated-histories
```

---
This document outlines all the steps taken during the GitHub setup and merging branches.

