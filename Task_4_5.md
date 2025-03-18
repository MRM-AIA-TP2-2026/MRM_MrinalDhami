# Resolving Git Merge and Push Issues

## Issue: Unable to Merge `Temp` Branch

### **Problem Encountered**
```bash
cobaltcheese@pop-os:~/Desktop/MRM/Taskfiles$ git merge Temp
merge: Temp - not something we can merge
```
### **Solution**
This error indicates that Git does not recognize `Temp` as a valid branch for merging. Ensure the branch exists locally and remotely:

1. **Check Local and Remote Branches:**
   ```bash
   git branch --all
   ```
   - If `Temp` does not exist locally, you need to create or fetch it.

2. **Fetch Remote Branches:**
   ```bash
   git fetch origin
   ```

3. **Ensure You Have the Latest Changes:**
   ```bash
   git checkout Temp
   git merge origin/Temp
   ```

4. **Switch to `master` and Merge `Temp`:**
   ```bash
   git checkout master
   git merge Temp
   ```

5. **Push Merged Changes to Remote:**
   ```bash
   git push origin master
   ```

---

## Issue: Push Rejected Due to Outdated Local `Temp` Branch

### **Problem Encountered**
```bash
cobaltcheese@pop-os:~/Desktop/MRM/Taskfiles$ git push -u origin Temp
Username for 'https://github.com': Nalla-Turing
Password for 'https://Nalla-Turing@github.com':
To https://github.com/MRM-AIA-TP2-2026/MRM_MrinalDhami.git
 ! [rejected]        Temp -> Temp (fetch first)
error: failed to push some refs to 'https://github.com/MRM-AIA-TP2-2026/MRM_MrinalDhami.git'
hint: Updates were rejected because the remote contains work that you do
hint: not have locally. This is usually caused by another repository pushing
hint: to the same ref. You may want to first integrate the remote changes
hint: (e.g., 'git pull ...') before pushing again.
hint: See the 'Note about fast-forwards' in 'git push --help' for details.
```
### **Solution**
This error happens because the remote branch `Temp` has updates that your local branch does not. To fix this:

1. **Fetch the Latest Changes:**
   ```bash
   git fetch origin
   ```

2. **Switch to the `Temp` Branch:**
   ```bash
   git checkout Temp
   ```

3. **Merge Remote Changes:**
   ```bash
   git merge origin/Temp
   ```
   - If there are conflicts, resolve them manually, then commit:
     ```bash
     git add .
     git commit -m "Resolved merge conflicts"
     ```

4. **Push the Updated `Temp` Branch to Remote:**
   ```bash
   git push origin Temp
   ```

After these steps, the `Temp` branch should be successfully merged and pushed. If further issues arise, check `git status` for additional context.

