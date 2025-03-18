# Adding Employee File to GitHub Repository

## **Step 1: Download the File**
```bash
wget -O Employee.txt "https://app.superthread.com/mrmaiatp22026/card-23-github?attachmentId=36a630c4-3e47-4111-bbfb-03341e61da9e"
```
This downloads the file and saves it as `Employee.txt`.

## **Step 2: Rename the File**
```bash
mv Employee.txt Employee.csv
```
This changes the file extension from `.txt` to `.csv`.

## **Step 3: Add the File to Git**
```bash
git add Employee.csv
```
This stages the file for commit.

## **Step 4: Commit the Changes**
```bash
git commit -m "Added Employee.csv to repository"
```
This saves the changes in your local repository.

## **Step 5: Push the Changes to GitHub**
```bash
git push origin master
```
This uploads the file to your GitHub repository.

---

These steps ensure that the `Employee.csv` file is successfully added and pushed to the repository.

