First, [create an account](https://github.com/join). Use your school email to get GitHub Student Developer Pack benefits. Then, install [Git](https://git-scm.com/download) and, for GitHub integration with Visual Studio Code, install [GitHub Pull Requests and Issues](https://marketplace.visualstudio.com/items?itemName=GitHub.vscode-pull-request-github). Then, configure everything (it should show notifications to help you with this.)

## Setting up Git

1. Open the terminal 
>![](./.images/terminal.png)

2. Ensure you're in the right directory. With `cmd`, it should appear next to your cursor. With `powershell`, you can use `pwd` to print the current directory. It should have some variation of `FRC2020` in it. If it doesn't, use `cd` to change to the right directory. You can use `ls` to list the contents of the current directory.

3. Run `git config --global user.name "Your Name"` to set your name. Replace "Your Name" with your name.

4. Run `git config --global user.email "your.email@uaschools.org"` to set your email. Replace "your.email" with your email.

After this, you have 2 options: **Upload your current workspace to the repository** or **Clone the repository to your current workspace**. The second option will start you fresh, but will get rid of your current code.

### Uploading your current workspace to the repository (recommended)

Now that you've set everything up, you'll want to upload your current workspace to the repository. This will allow you to work with your team members and keep track of your code and its latest changes.

1. Save everything in your workspace. You can do this by pressing `Ctrl + K then S` or by clicking `File (menu) > Save All` in the top left corner of the screen.
2. Create a new repository locally. This keeps all your code and allows you to upload it to GitHub. Run `git init` in the terminal. This will create a new folder called `.git` in your workspace. This folder contains all the information about your repository.