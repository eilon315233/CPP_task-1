משתמש יקר, 

זו היא מערכת אשר מקבלת גרפים בתצורה של מטריצת שכנויות, וממשת פעולות שונות על הגרף.

במערכת זו ישנם מספר קבצים:

---קובץ Graph.hpp: אשר מכיל בתוכו את התכונות הבסיסיות של גרף ואת החתימות של הפעולות הבסיסיות שאפשר לקיים על גרף.

---קובץ Graph.cpp: אשר מכיל בתוכו את כל המימושים של הפעולות השונות בקובץ Graph.hpp.

---קובץ Algorithms.hpp: אשר מכיל בתוכו את כל החתימות של האלגוריתמים השונים שאפשר לממש על גרף.

---קובץ Algorithms.cpp: אשר מכיל בתוכו את כל המימושים של האלגוריתמים השונים בקובץ Algorithms.hpp.

---קובץ Demo.cpp: זו הוא הקובץ המרכזי שיריץ מספר פעולות שונות על גרפים שונים שכבר הוכנסו מראש, וידפיס את התוצאות שלהם.

---קובץ Test.cpp: זו הוא קובץ שמכיל בדיקות שונות על פעולות שונות על גרפים, אשר בודק אם הם עובדות נכון.

---קובץ TestCounter.cpp:

---קובץ doctest.h: קובץ



אפרט על הפעולות המרכזיות שתוכל לממש על גרף:

<פעולות בסיסיות>

---פעולה 1 - loadGraph: פעולה זו מאפשרת לך לטעון גרף חדש למערכת (בתצורה של מטריצת שכנויות), ואח"כ תבדוק האם הגרף שהזנת הינו מכוון או לא, ותשמור לך תכונה זו.

---פעולה 2 - printGraph: פעולה זו תדפיס את התצורה של הגרף ואת מספר הקודקודים והצלעות שבו.

<אלגוריתמים שונים>

---אלגוריתם 1 -isConnected: 

אלגוריתם זה בודק האם הגרף שהזנת קשיר או לא, בעזרת מבנה הנתונים vector עם ערכים בוליאנים, והאלגוריתם dfs.
בעזרת DFS התוכנית תעבור על כל הגרף שהוכנס מקודקוד מסוים, ותנסה להגיע לכל שאר קודקודי הגרף, במידה וישאר קודקוד אחד שלא הגיעו אליו, הוא ישאר עם ערך false, ויודפס "0" - אשר מעיד על גרף שאינו קשיר.
במידה והגרף שהוכנס היה ריק - יוחזר "0" - אשר מעיד על גרף שאינו קשיר.
אחרת - יוחזר "1" - אשר מעיד על גרף קשיר.
 

---אלגוריתם 2 - shortestPath:

אלגוריתם זה מקבל 2 קודקודים בגרף ומחזיר את הדרך הקצרה ביותר מקודקוד ההתחלה לקודקוד הסיום, במידה ואין מסלול - התוכנית תחזיר "1-".

---אלגוריתם 3 -isContainsCycle:

אלגוריתם זה בודק האם קיים מעגל בגרף ( מסלול שמתחיל בקודקוד מסוים ומסיים באותו קודקוד) ומדפיס אותו, אם אין אחד כזה - התוכנית תחזיר "0".

---אלגוריתם 4 - isBipartite:

אלגוריתם זה בודק האם הגרף "דו צדדי", במידה וכן - ידפיס את החלוקה שלו לשני קבוצות הקודקודים, במידה ואין אפשרות כזאת - התוכנית תחזיר "0".

---אלגוריתם 5 -  negativeCycle:

האלגוריתם יבדוק האם קיים מעגל שסכום משקליו הוא שלילי, במידה וכן - היא תדפיס אותו, במידה ולא - היא תחזיר "לא קיים מעגל שלילי בגרף".



