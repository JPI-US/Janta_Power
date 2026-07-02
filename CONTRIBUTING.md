# Contributing

## Branch Structure

* **Master** – Production branch. Code deployed to customer towers that has been fully tested and proven stable.
* **Deployment** – Pre-production branch. Code currently being deployed to customer towers. Once it has been validated as stable, merge it into **Master**.
* **Testing** – Active testing branch. Code undergoing validation in a controlled environment on our towers. After the testing cycle is complete, merge it into **Deployment**.
* **Pre-testing** – Integration branch for approved features that are not part of the current testing cycle. Merge this branch into **Testing** when the next testing cycle begins.
* **Feature Branches** – Branches for new features and changes under development. After review and approval, merge them into **Pre-testing**.