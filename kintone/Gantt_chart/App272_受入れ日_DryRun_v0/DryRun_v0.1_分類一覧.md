# DryRun v0.1 分類一覧

| Class | 意味 | 将来自動書込み候補 |
|---|---|---|
| `CANDIDATE_EMPTY_SINGLE` | App272空欄・単一受入 | YES |
| `CANDIDATE_EMPTY_MULTI` | App272空欄・複数受入 | REVIEW |
| `ALREADY_SAME` | App272既存値と候補日一致 | 不要 |
| `EXISTING_DIFFERENT` | App272既存値と候補日不一致 | NO / 人確認 |
| `SOURCE_REVIEW_REQUIRED` | 負数受入等、ソース解釈要 | NO |
| `SOURCE_ONLY_NO_APP272` | 受入実績にはあるがApp272なし | NO |
| `NO_SOURCE_DATA_UNKNOWN` | App272にはあるが今回の受入実績に無い | NO / 未受入とは断定禁止 |

## 絶対ルール

`NO_SOURCE_DATA_UNKNOWN != 未受入`

基幹システムへの受入入力漏れが一定数存在する可能性があるため、
自動処理だけで100%の受入状況を保証しない。
