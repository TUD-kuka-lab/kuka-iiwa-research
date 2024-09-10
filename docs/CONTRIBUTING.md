*Development status table:* [table](https://docs.google.com/spreadsheets/d/1DTmn35bPQ2DiaHUpZ2mUlgpueO4ZZamep00HAa-jS6c/edit?usp=sharing)

Bug reports, feature requests or code contributions are always very welcome.

To make things easier, here are a few tips:

Inside CoR
-----------------------------------
Branch and upload your modifications submit a PR:
```sh
git checkout -b <my_branch>
#do your modifications
git add <modified_files>
git commint -m "<drescriptive message>"
git push orgin <my_branch>
```

Now access the remote to create the PR:
1. go to the [repo](https://gitlab.tudelft.nl/kuka-iiwa-7-cor-lab/iiwa_ros)
2. go to "merge requests" and create a new merge request ![image](https://gitlab.tudelft.nl/kuka-iiwa-7-cor-lab/iiwa_ros/-/blob/master/docs/merge.png)
3. select <my_branch> as source branch
4. select the target branch (usually master)
5. **Assign** the merge request to a reviewer. Squashing and deleting the source branch are strongly recommended.

Directly to LASA:
-----------------

Reporting bugs, requesting features
-----------------------------------

-   Best way to report bugs and request new features is to use GitHub
    [Issues](https://github.com/epfl-lasa/iiwa_ros/issues), but you can contact the maintainers also any other way — see the [README](README.md) for details.

Code contribution
-----------------

-   Best way to contribute is using GitHub [Pull Requests](https://github.com/epfl-lasa/iiwa_ros/pulls)
    — fork the repository and make a pull request from a feature branch. You can also send patches via e-mail or contact the maintainers in any other way — see the [README](README.md) for details.
-   Follow the project coding guidelines. In case of doubt, please contact the maintainers.
-   All your code will be released under the project license (see the [LICENSE](LICENSE) file for details), so make sure you and your collaborators (or employers) have no problems with it.
